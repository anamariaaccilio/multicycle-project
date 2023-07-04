
module alu(a,b,ALUControl,Result,ALUFlags, clk);
  input clk; 
  input [31:0] a,b;
  input [2:0] ALUControl; 
  reg [31:0] R1_mull; 
  reg [31:0] R2_mull; 
  output reg [31:0] Result; 
  reg [31:0] Result_reg;
  output wire [3:0] ALUFlags; 


  wire [31:0] condinvb;
  wire [32:0] sum;
  wire [63:0] R_mull; 
  

  
  assign condinvb = ALUControl[0] ? ~b : b;
  assign sum = a + condinvb + ALUControl[0];

  /*
  assign mull = a + condinvb + ALUControl[0];  //modificado
        3'b011: Result = a | b
	  endcase
    end
  */

  // Maneja lógica secuencial


  
  always @(posedge clk)
    begin
	   
	   R1_mull <= a;
	   R2_mull <= b; 

	   casex(ALUControl[2:0])

	    3'b00?: Result_reg = sum;
	    3'b010: Result_reg = a & b; 
	    3'b011: Result_reg = a | b; 
	    3'b111: Result_reg = R_mull[31:0];
	   endcase

	   //mod
	   //verificar el assign


       /*

	   neg = Result[31]; 
	   zero = (Result == 32'b0);
	   carry = (ALUControl[1] == 1'b0) & sum[32]; 
	   overflow = (ALUControl[1] == 1'b0) & ~(a[31] ^ b[31] ^ ALUControl[0]) & (a[31] ^ sum[31]);
       */
	   
    end
  
  always @(posedge clk)
    begin 
	    case (ALUControl)
            3'b000: 
              begin
                R_mull = R1_mull * R2_mull; //modificado - MUL
                neg = R_mull[31];
                zero = (R_mull == 32'b0);
                carry = 1'b0;
                overflow = 1'b0;
              end
            3'b100:
              begin
                R_mull = R1_mull * R2_mull; //modificado - MULL
                neg = R_mull[31];
                zero = (R_mull == 32'b0);
                carry = 1'b0;
                overflow = 1'b0;
              end
            3'b110:
              begin
                R_mull = $signed(R1_mull) * $signed(R2_mull); //SMULL
                neg = R_mull[31];
                zero = (R_mull == 32'b0);
                carry = 1'b0;
                overflow = 1'b0;
              end
            default:
              begin
                R_mull = 64'b0;
                neg = Result[31];
                zero = (Result == 32'b0);
                carry = (ALUConstrol[1] == 1'b0) & sum[32];
                overflow = (ALUControl[1] == 1'b0) & ~(a[31] ^ b[31] ^ ALUControl[0]) & (a[31] ^ sum[31]);
              end
        endcase
	end

  always @(posedge clk)//clk 
    begin
        Result <= Result_reg; 
	end 

	
   assign neg = Result[31]; 
   assign zero = (Result == 32'b0);
   assign carry = (ALUControl[1] == 1'b0) & sum[32];
   assign overflow = (ALUControl[1] == 1'b0) & ~(a[31] ^ b[31] ^ ALUControl[0]) & (a[31] ^ sum[31]);
   assign ALUFlags = {neg, zero, carry, overflow};
	

	//NOTA: EL TIPO DE LÓGICA ES SECUENCIAL PORQUE LUEGO SE ESCRIBE 
	
endmodule

