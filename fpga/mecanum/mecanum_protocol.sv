// 24 bytes: A5 5A 02 op seq motor_mask encoder_mask 00,
// x/y/z int16, kp/ki uint16, four reserved zero bytes, CRC16 little endian.
module mc_mecanum_protocol #(parameter CLK_HZ=50000000,VERSION=2)(
 input wire clk,reset_n,byte_valid,framing_error,input wire [7:0] byte_data,
 output reg valid,output reg [175:0] body,output reg [31:0] bad_frames
);
 reg [175:0] pending;
 reg [4:0] index;
 reg [15:0] crc;
 reg [7:0] low;
 integer idle;
 function automatic [15:0] update(input [15:0] old,input [7:0] v);
 reg [15:0] c;integer j;
 begin c=old^{v,8'b0};for(j=0;j<8;j=j+1)c=c[15]?(c<<1)^16'h1021:c<<1;update=c;end
 endfunction
 always @(posedge clk or negedge reset_n) begin
  if(!reset_n)begin pending<=0;body<=0;index<=0;crc<=16'hffff;low<=0;idle<=0;valid<=0;bad_frames<=0;end
  else begin
   valid<=0;
   if(index!=0)idle<=idle+1;
   if(framing_error || (index!=0 && idle>=CLK_HZ/50))begin index<=0;idle<=0;bad_frames<=bad_frames+1;end
   else if(byte_valid)begin
    idle<=0;
    if(index==0)begin
     if(byte_data==8'ha5)begin pending[7:0]<=byte_data;crc<=update(16'hffff,byte_data);index<=1;end
    end else if(index==1 && byte_data!=8'h5a)begin
     if(byte_data==8'ha5)crc<=update(16'hffff,byte_data);else index<=0;
    end else if(index<22)begin pending[index*8+:8]<=byte_data;crc<=update(crc,byte_data);index<=index+1'b1;end
    else if(index==22)begin low<=byte_data;index<=23;end
    else begin
     index<=0;
     if({byte_data,low}==crc && pending[23:16]==VERSION && pending[63:56]==0 &&
        (pending[175:144]==0 || pending[31:24]==5 || pending[31:24]==6 || pending[31:24]==9) &&
        pending[47:44]==0 && pending[55:52]==0)begin body<=pending;valid<=1;end
     else bad_frames<=bad_frames+1;
    end
   end
  end
 end
endmodule
