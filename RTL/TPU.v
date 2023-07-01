

//`include "unit.v"
module TPU(
    clk,
    rst_n,

    in_valid,
    K,
    M,
    N,
    busy,

    A_wr_en,
    A_index,
    A_data_in,
    A_data_out,

    B_wr_en,
    B_index,
    B_data_in,
    B_data_out,

    C_wr_en,
    C_index,
    C_data_in,
    C_data_out
);


input clk;
input rst_n;
input            in_valid;
input [7:0]      K;
input [7:0]      M;
input [7:0]      N;
output  reg      busy;

output           A_wr_en;
output [15:0]    A_index;
output [31:0]    A_data_in;
input  [31:0]    A_data_out;

output           B_wr_en;
output [15:0]    B_index;
output [31:0]    B_data_in;
input  [31:0]    B_data_out;
 
output           C_wr_en;
output [15:0]    C_index;
output [127:0]   C_data_in;
input  [127:0]   C_data_out;

//* Implement your design here
//(M,K)*(K,N)

//---------------initial----------------------------------//
reg [8:0]   K_copy,M_copy,N_copy,K_mem,N_mem,M_mem;
reg[2:0]    state,next_state;
reg[5:0]    i;

reg [7:0]matrix_A1[3:0];
reg [7:0]matrix_A2[3:0];
reg [7:0]matrix_A3[3:0];
reg [7:0]matrix_A4[3:0];

reg [7:0]matrix_B1[3:0];
reg [7:0]matrix_B2[3:0];
reg [7:0]matrix_B3[3:0];
reg [7:0]matrix_B4[3:0];

//buffer for left input A
reg [7:0] A_line1[3:0];//789 for pe register
reg [7:0] A_line2[3:0];
reg [7:0] A_line3[3:0];
reg [7:0] A_line4[3:0];

//buffer for left input B
reg [7:0] B_line1[3:0];//789 for pe register
reg [7:0] B_line2[3:0];
reg [7:0] B_line3[3:0];
reg [7:0] B_line4[3:0];

//control signal
reg[15:0] cycle;//caculate load and exe cycle
reg exe;//to signal starting compute
reg reload;//to load remaining data
reg [1:0]last_row;//signal the last row of K in the next turn to fix c_temp
reg last_K;//to reset matrix
reg stored;//signal that answer is stored and need to clear buffer
reg [7:0]N_num;//record the times N_copy decrease to 0
reg [2:0]K_fixed;//load state will let index increase 4, but sometimes we dont need to load 4 stages
reg [2:0]M_fixed;//fixed the remaining padding 0 line to c_index
reg [1:0]M_last_row;//signal the last row of M
reg load_first_3cycle;
reg [3:0]load_last_3cycle;
reg transfer_pe;
reg load;//load data
reg [1:0]load_index;
reg the_last_index;
reg [20:0]a_temp;//for address index a
reg [20:0]b_temp;
reg [20:0]c_temp;

parameter init_state = 'd0;
parameter load_data = 'd1;
parameter caculate = 'd2;
parameter store_data = 'd3;

wire [7:0]con_a[11:0];//connect pe
wire [7:0]con_b[11:0];
wire [31:0]con_c[15:0];

reg [127:0]c_data[3:0];//store outcome
//fsm
always@(*)begin
    case(state)
        init_state: begin
            transfer_pe = 0;
            exe = 0;
            cycle = 1;
            a_temp = 0;b_temp = 0;c_temp = 0;//index reset
            stored = 0;
            reload = 'd1;
            N_num = 0;
            last_row = 0;//fixed c_temp
            M_last_row = 0;
            last_K = 0;
            load_first_3cycle = 1;
            load_last_3cycle = 0;//reset
            load = 1;//load data control
            load_index = 0;//control matrix index
            the_last_index = 1;//1 haven't load the last index, 0 already load the last index
            for(i=0;i<=3;i=i+1)begin
                matrix_A1[i] <= 'd0;
                matrix_A2[i] <= 'd0;
                matrix_A3[i] <= 'd0;
                matrix_A4[i] <= 'd0;
                matrix_B1[i] <= 'd0;
                matrix_B2[i] <= 'd0;
                matrix_B3[i] <= 'd0;
                matrix_B4[i] <= 'd0;
            end
            next_state = in_valid? load_data:init_state;//invaild = 1, next_state = load data
        end
        load_data : begin//load the first 3 cycle
            transfer_pe = 1'd1;//start pe transfer
            if(cycle == 'd4 )begin
                // cycle = 0;
                exe = 1'd1;//go to compute states   
                load_first_3cycle = 0;
            end else begin
                exe = 1'd0;// not go to compute state
            end
            if(last_row)begin
                c_temp = c_temp - 1;//the store index increase 4 repeatly, but sometimes the reamining line is padding to 0 that need not to stored
                last_row = 0;//reset 
                last_K = 0;
                for(i=0;i<=3;i=i+1)begin
                   c_data[i]=128'd0;                    
                 end
            end 
            if(M_last_row)begin
                c_temp <= c_temp - M_fixed;//fixed storing 0
                M_last_row = 0;//reset 
            end
            next_state = exe? caculate: load_data;
        end
        caculate : begin
            if(K_copy == 0)begin
                load = 0;// no new data
                the_last_index = 0;//already load the last index
                if(load_last_3cycle == 4)begin // for computing, load the last 3 cycle
                    cycle = 0;//reset and jump to stored data
                    next_state <= store_data;
                    stored = 1;// keep store
                end
            end else begin
                exe = 1'd0;//reset load state control
                next_state <= caculate;
            end 
        end
        store_data : begin
            if(cycle == 4)
                transfer_pe = 0;//clear pe buff
            if(cycle == 5)begin
                if(N_copy == 0)begin
                    busy = 1'b0;
                end
                stored = 0;
                reload = 1;
                load = 1;//reset 
                load_last_3cycle = 0;//reset
                transfer_pe = 1;//reset
                the_last_index = 1;
            end
            if(M_copy <= 0)begin
                M_last_row <= 1;
            end
            if(stored)begin
                next_state = store_data;//keep store data
            end else begin
                if(busy)begin//busy = 1
                    next_state = load_data;// keep load data again
                    last_row = 1;
                    cycle = 1;
                end else begin//busy = 0 check answer
                    next_state = init_state;              
                end
            end
        end
    endcase
end

always@(posedge clk or negedge rst_n)begin
    if(state == store_data)begin
        if(K_copy <= 0)begin
            M_copy <= M_copy - 'd4;
            K_copy <= K_mem;//reset K
            b_temp <= (N_num * K_mem) + 1;//reload matix B
    end else begin
        if(M_copy <= 0)begin//load all matrix A finish
            N_num <= N_num + 1;
            a_temp <= 1;//reload matrix B
            M_copy <= M_mem;//reset M
            N_copy <= N_copy - 'd4;
            b_temp <= ((1+N_num) * K_mem) + 1;
        end    
    end
    end
end

always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        state <= init_state;
        busy <= 1'b0;
    end
    else begin
        state <= next_state;
        busy <= (in_valid) ? 1'b1 : busy;
    end
end

assign A_wr_en = 1'd0;//assign read signal
assign B_wr_en = 1'd0;
assign A_index = a_temp - 1;//assign address
assign B_index = b_temp - 1;

//clear buffer for a and b
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        for(i=0;i<=3;i=i+1)begin
            A_line1[i] <=8'd0;
            A_line2[i] <=8'd0;
            A_line3[i] <=8'd0;
            A_line4[i] <=8'd0;
            B_line1[i] <=8'd0;
            B_line2[i] <=8'd0;
            B_line3[i] <=8'd0;
            B_line4[i] <=8'd0;
        end
    end else begin
        if(in_valid)begin
            for(i=0;i<=3;i=i+1)begin
            A_line1[i] <=8'd0;
            A_line2[i] <=8'd0;
            A_line3[i] <=8'd0;
            A_line4[i] <=8'd0;
            B_line1[i] <=8'd0;
            B_line2[i] <=8'd0;
            B_line3[i] <=8'd0;
            B_line4[i] <=8'd0;
        end
        c_data[0]<= 128'd0;
        c_data[1]<= 128'd0;
        c_data[2]<= 128'd0;
        c_data[3]<= 128'd0;

        end    
    end
end

//save parameter KMN
always@(posedge clk or negedge rst_n)begin 
    if(!rst_n)begin 
        K_copy <= 1'd0;
        M_copy <= 1'd0;
        N_copy <= 1'd0;
    end else begin
        if(in_valid)begin
            if(K < 4)begin
                K_copy <= 4;
            end else begin
                K_copy <= K;
                K_mem <= K;
                case((K%4))
                    0: K_fixed <= 0;
                    default: K_fixed <= 1;//load would run more 1 cycle if need not to load 4 data
                endcase
            end
            if(M < 4)begin
                M_copy <= 4;
            end else begin//make M a mutiple of 4
                case((M%4))//how many line are padding to 0 (4-(M%4))
                    0: begin
                        M_fixed <= 0;M_copy <= M ;M_mem <= M;                       
                    end
                    1: begin
                        M_fixed <= 3;M_copy <= M + 3;M_mem <= M + 3;                        
                    end
                    2: begin
                        M_fixed <= 2;M_copy <= M + 2;M_mem <= M + 2;    
                    end
                    3: begin
                        M_fixed <= 1;M_copy <= M + 1;M_mem <= M + 1;                        
                    end
                endcase
            end
            if(N < 4)begin
                N_copy <= 4;
            end else begin//make N a mutiple of 4
                case((N%4))//how many line are padding to 0 (4-(M%4))
                    0: begin
                        N_copy <= N ;N_mem <= N;                       
                    end
                    1: begin
                        N_copy <= N + 3;N_mem <= N + 3;                       
                    end
                    2: begin
                        N_copy <= N + 2; N_mem <= N + 2;                      
                    end
                    3: begin
                        N_copy <= N + 1; N_mem <= N + 1;                       
                    end
                endcase
            end
        end
    end
end
//-----------------read data-----------------------------//
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        cycle <= 0;
    end else begin
        if(load)begin
            a_temp <= a_temp + 1'd1;
            b_temp <= b_temp + 1'd1;
            load_index <= load_index + 2'd1;//1234
            K_copy = K_copy -1;                    
            matrix_A1[((cycle-1)%4)] <= A_data_out[31:24];//11
            matrix_A2[((cycle-1)%4)] <= A_data_out[23:16];//12
            matrix_A3[((cycle-1)%4)] <= A_data_out[15:8];//13
            matrix_A4[((cycle-1)%4)] <= A_data_out[7:0];//14
                            
            matrix_B1[((cycle-1)%4)] <= B_data_out[31:24];
            matrix_B2[((cycle-1)%4)] <= B_data_out[23:16];
            matrix_B3[((cycle-1)%4)] <= B_data_out[15:8];
            matrix_B4[((cycle-1)%4)] <= B_data_out[7:0];         
        end else begin
            load_last_3cycle <= load_last_3cycle + 'd1;
            matrix_A1[((cycle-1)%4)] <= 'd0;//11
            matrix_A2[((cycle-1)%4)] <= 'd0;//12
            matrix_A3[((cycle-1)%4)] <= 'd0;//13
            matrix_A4[((cycle-1)%4)] <= 'd0;//14
                            
            matrix_B1[((cycle-1)%4)] <= 'd0;
            matrix_B2[((cycle-1)%4)] <= 'd0;
            matrix_B3[((cycle-1)%4)] <= 'd0;
            matrix_B4[((cycle-1)%4)] <= 'd0; 
        end
    end    
end
//reset
always@(posedge clk or negedge rst_n)begin
    if(last_K)begin
        for(i=0;i<=3;i=i+1)begin
            matrix_A1[i] <= 'd0;
            matrix_A2[i] <= 'd0;
            matrix_A3[i] <= 'd0;
            matrix_A4[i] <= 'd0;
            matrix_B1[i] <= 'd0;
            matrix_B2[i] <= 'd0;
            matrix_B3[i] <= 'd0;
            matrix_B4[i] <= 'd0; 
        end
        last_K = 0;    
    end          
end
//---------------compute stage----------------------------------//
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        cycle <= 0;
    end else begin
        if(transfer_pe == 1)begin
            cycle <= cycle + 1;
            if(cycle <= 4)begin
                case(cycle)
                    1:begin
                        if(!the_last_index)begin//the_last_index = 0
                            A_line1[0] <= 'd0;
                        end else begin
                            A_line1[0] <= A_data_out[31:24];//11
                        end//11
                        A_line2[0] <= 'd0;
                        A_line3[0] <= 'd0;
                        A_line4[0] <= 'd0;

                        if(!the_last_index)begin
                            B_line1[0] <= 'd0;
                        end else begin
                            B_line1[0] <= B_data_out[31:24];//11
                        end
                        B_line2[0] <= 'd0;
                        B_line3[0] <= 'd0;
                        B_line4[0] <= 'd0;
                    end
                    2:begin
                        if(!the_last_index)begin//the_last_index = 0
                            A_line1[0] <= 'd0;
                        end else begin
                            A_line1[0] <= A_data_out[31:24];//11
                        end
                        A_line2[0] <= matrix_A2[0];
                        A_line3[0] <= 'd0;
                        A_line4[0] <= 'd0;

                        if(!the_last_index)begin
                            B_line1[0] <= 'd0;
                        end else begin
                            B_line1[0] <= B_data_out[31:24];//11
                        end
                        B_line2[0] <= matrix_B2[0];
                        B_line3[0] <= 'd0;
                        B_line4[0] <= 'd0;
                    end
                    3:begin
                        if(!the_last_index)begin//the_last_index = 0
                            A_line1[0] <= 'd0;
                        end else begin
                            A_line1[0] <= A_data_out[31:24];//11
                        end
                        A_line2[0] <= matrix_A2[1];
                        A_line3[0] <= matrix_A3[0];
                        A_line4[0] <= 'd0;

                        if(!the_last_index)begin
                        B_line1[0] <= 'd0;
                        end else begin
                            B_line1[0] <= B_data_out[31:24];//11
                        end
                        B_line2[0] <= matrix_B2[1];
                        B_line3[0] <= matrix_B3[0];
                        B_line4[0] <= 'd0;
                    end
                    4:begin
                        if(!the_last_index)begin//the_last_index = 0
                            A_line1[0] <= 'd0;
                        end else begin
                            A_line1[0] <= A_data_out[31:24];//11
                        end
                        A_line2[0] <= matrix_A2[2];
                        A_line3[0] <= matrix_A3[1];
                        A_line4[0] <= matrix_A4[0];
    
                        if(!the_last_index)begin
                            B_line1[0] <= 'd0;
                        end else begin
                            B_line1[0] <= B_data_out[31:24];//11
                        end
                        B_line2[0] <= matrix_B2[2];
                        B_line3[0] <= matrix_B3[1];
                        B_line4[0] <= matrix_B4[0];
                end
                endcase
            end else begin
                    if(!the_last_index)begin//the_last_index = 0
                        A_line1[0] <= 'd0;
                    end else begin
                        A_line1[0] <= A_data_out[31:24];//11
                    end
                    A_line2[0] <= matrix_A2[((cycle-2)%4)];
                    A_line3[0] <= matrix_A3[((cycle-3)%4)];
                    A_line4[0] <= matrix_A4[((cycle-4)%4)];
                    
                    if(!the_last_index)begin
                        B_line1[0] <= 'd0;
                    end else begin
                        B_line1[0] <= B_data_out[31:24];//11
                    end
                    B_line2[0] <= matrix_B2[((cycle-2)%4)];
                    B_line3[0] <= matrix_B3[((cycle-3)%4)];
                    B_line4[0] <= matrix_B4[((cycle-4)%4)];
            end
  
            A_line1[1]<= con_a[0];A_line1[2]<= con_a[1];A_line1[3]<= con_a[2]; 
            A_line2[1]<= con_a[3];A_line2[2]<= con_a[4];A_line2[3]<= con_a[5]; 
            A_line3[1]<= con_a[6];A_line3[2]<= con_a[7];A_line3[3]<= con_a[8]; 
            A_line4[1]<= con_a[9];A_line4[2]<= con_a[10];A_line4[3]<= con_a[11]; 
    
            B_line1[1]<= con_b[0];B_line1[2]<= con_b[1];B_line1[3]<= con_b[2]; 
            B_line2[1]<= con_b[3];B_line2[2]<= con_b[4];B_line2[3]<= con_b[5]; 
            B_line3[1]<= con_b[6];B_line3[2]<= con_b[7];B_line3[3]<= con_b[8]; 
            B_line4[1]<= con_b[9];B_line4[2]<= con_b[10];B_line4[3]<= con_b[11]; 

            if(K_copy == 0)begin
                c_data[0] <= {con_c[0],con_c[1],con_c[2],con_c[3]};
           end

        end
    end
end

//pe
unit pe11(.a_in(A_line1[0]), .b_in(B_line1[0]), .state(transfer_pe), .clk(clk), .rst_n(rst_n), .a_out(con_a[0]), .b_out(con_b[0]), .c_out(con_c[0]));
unit pe12(.a_in(A_line1[1]), .b_in(B_line2[0]),  .state(transfer_pe),.clk(clk), .rst_n(rst_n), .a_out(con_a[1]), .b_out(con_b[3]), .c_out(con_c[1]));
unit pe13(.a_in(A_line1[2]), .b_in(B_line3[0]), .state(transfer_pe),.clk(clk), .rst_n(rst_n), .a_out(con_a[2]), .b_out(con_b[6]), .c_out(con_c[2]));
unit pe14(.a_in(A_line1[3]), .b_in(B_line4[0]), .state(transfer_pe),.clk(clk), .rst_n(rst_n), .a_out(), .b_out(con_b[9]), .c_out(con_c[3]));
unit pe21(.a_in(A_line2[0]), .b_in(B_line1[1]), .state(transfer_pe),.clk(clk), .rst_n(rst_n), .a_out(con_a[3]), .b_out(con_b[1]), .c_out(con_c[4]));
unit pe22(.a_in(A_line2[1]), .b_in(B_line2[1]),  .state(transfer_pe),.clk(clk), .rst_n(rst_n), .a_out(con_a[4]), .b_out(con_b[4]), .c_out(con_c[5]));
unit pe23(.a_in(A_line2[2]), .b_in(B_line3[1]),  .state(transfer_pe),.clk(clk), .rst_n(rst_n), .a_out(con_a[5]), .b_out(con_b[7]), .c_out(con_c[6]));
unit pe24(.a_in(A_line2[3]), .b_in(B_line4[1]), .state(transfer_pe),.clk(clk), .rst_n(rst_n), .a_out(), .b_out(con_b[10]), .c_out(con_c[7]));
unit pe31(.a_in(A_line3[0]), .b_in(B_line1[2]), .state(transfer_pe),.clk(clk), .rst_n(rst_n), .a_out(con_a[6]), .b_out(con_b[2]), .c_out(con_c[8]));
unit pe32(.a_in(A_line3[1]), .b_in(B_line2[2]),  .state(transfer_pe),.clk(clk), .rst_n(rst_n), .a_out(con_a[7]), .b_out(con_b[5]), .c_out(con_c[9]));
unit pe33(.a_in(A_line3[2]), .b_in(B_line3[2]),  .state(transfer_pe),.clk(clk), .rst_n(rst_n), .a_out(con_a[8]), .b_out(con_b[8]), .c_out(con_c[10]));
unit pe34(.a_in(A_line3[3]), .b_in(B_line4[2]),  .state(transfer_pe),.clk(clk), .rst_n(rst_n), .a_out(), .b_out(con_b[11]), .c_out(con_c[11]));
unit pe41(.a_in(A_line4[0]), .b_in(B_line1[3]), .state(transfer_pe),.clk(clk), .rst_n(rst_n), .a_out(con_a[9]), .b_out(), .c_out(con_c[12]));
unit pe42(.a_in(A_line4[1]), .b_in(B_line2[3]), .state(transfer_pe),.clk(clk), .rst_n(rst_n), .a_out(con_a[10]), .b_out(), .c_out(con_c[13]));
unit pe43(.a_in(A_line4[2]), .b_in(B_line3[3]), .state(transfer_pe),.clk(clk), .rst_n(rst_n), .a_out(con_a[11]), .b_out(), .c_out(con_c[14]));
unit pe44(.a_in(A_line4[3]), .b_in(B_line4[3]), .state(transfer_pe),.clk(clk), .rst_n(rst_n), .a_out(), .b_out(), .c_out(con_c[15]));

//---------------store data back----------------------------------//
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin

    end else begin
        if(state == store_data)begin
            case(cycle)
                1 : c_data[1] <= {con_c[4],con_c[5],con_c[6],con_c[7]};
                2 : c_data[2] <= {con_c[8],con_c[9],con_c[10],con_c[11]};
                3 : c_data[3] <= {con_c[12],con_c[13],con_c[14],con_c[15]};
            endcase
            cycle <= cycle + 1;
            c_temp <= c_temp + 1;

        end
    end
end    
assign C_wr_en = 1'd1;
assign C_data_in = c_data[cycle-1];
assign C_index = c_temp;

endmodule

