//`define Data_size 8

module unit (a_in,b_in,state,
            clk,rst_n,
            a_out,b_out,c_out 
);

 input [7:0]a_in,b_in;
 input clk,rst_n;
 input [2:0]state;
 output[7:0]a_out,b_out;
 output [31:0]c_out;

 reg [31:0]result;

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin     //reset
        result <= 0;
    end else begin      //compute 
        if(state == 'd1)begin
            result <= result + (a_in*b_in);
//            a_out <= a_in;
//            b_out <= b_in;
        end else begin
            result <= 0;
        end
    end
end

assign a_out = a_in;
assign b_out = b_in;
assign c_out = result; 

endmodule