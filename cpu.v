`timescale 1s/100ms 

// ************************************************************************************************
//                                      8-bit ALU MODULE
// ************************************************************************************************

/*
    FORWAD module implements the FORWAD functional unit of the ALU. It forwards whatever the 8-bit value in data2 port directly to the output.
*/
module FORWARD (data2, result);
    // Port declarations
    input [7:0] data2;
    output [7:0] result;

    assign #1 result = data2;       // result is continuously driven by the data2 signal with a delay of 1 simulation time step
endmodule

/*
    ADD module implements the ADD functional unit of the ALU. It performs arithmetic addition of two 8-bit values on data1 and data2 and feeds the result to the output.
*/
module ADD (data1, data2, result);
    // Port declarations
    input [7:0] data1, data2;
    output [7: 0] result;

    assign #2 result = data1 + data2;// result is continuously driven by the data1 and data2 signals with a delay of 2 simulation time steps
endmodule

/*
    AND module implements the AND functional unit of the ALU. It performs bitwise AND between two 8-bit values on data1 and data2 and feeds the result to the output.
*/
module AND (data1, data2, result);
    // Port declarations
    input [7:0] data1, data2;
    output [7: 0] result;

    assign #1 result = data1 & data2;// result is continuously driven by the data1 and data2 signals with a delay of 1 simulation time step
endmodule

/*
    OR module implements the OR functional unit of the ALU. It performs bitwise OR between two 8-bit values fed into data1, data2 and outputs the result.
*/
module OR (data1, data2, result);
    // Port declarations
    input [7:0] data1, data2;
    output [7:0] result;

    assign #1 result = data1 | data2;// result is continuously driven by the data1 and data2 signals with a delay of 1 simulation time step
endmodule

/*
    MULT module implements the multiplier functional unit of the ALU. It performs binary multiplication between two 8-bit values fed into data1, data2 and outputs 
    the result.

    The partial product method of multiplication is used.

    Ex:
        (data2)     11010110 (-42)
        (data1)   x 00000011 (3)
                  ---------------
                    11010110    (Partial Product 1)
                   110101100    (Partial Product 2)
                  0000000000    (Partial Product 3)
                     ...                ...
                  ---------------
                    10000010 (-126)
*/
module MULT (data1, data2, result);
    // Port declarations
    input [7:0] data1, data2;
    output reg [7:0] result;
    
    reg [7:0] shifted;                  // To store the left shifted value (Partial Products) of the value at data2 port

    integer i;                          // Counter used for the loop

    always @ * begin
        #3
        result = 8'd0;                  // Initialize the result to zero
        shifted = data2;                // Initially no shift is applied to data2 (multiplicand)
        // 8 MUXes + 8 Adders
        for (i = 0; i < 8; i = i + 1) begin
            if (data1[i])               // If the current bit of data1 (Multiplier) is LOW, that bit doesn't contribute to the product
                result = result + shifted;
            shifted = {shifted, 1'b0};  // Re-route the wiring for left shifting
        end
    end
endmodule

/*
    SHIFT module implements both the Logical and Arithmetic Shift functional unit of the ALU. It performs both the logical left, right shifts and arithmetic left, right
    shifts on data1 by an immediate value given at data2 and outputs the result.
*/
module SHIFT (data1, data2, lshift, ashift);
    // Port declarations
    input [7:0] data1, data2;
    output reg [7:0] lshift, ashift;

    integer i;                                      // Counter used for the loop

    always @ * begin
        // Both left and right shift result buses are wired to zeros initially
        lshift = 8'd0;                              // Logical shift and arithmetic left shift results are wired to zeros initially
        ashift = {8{data1[7]}};                     // For sign extension purpose
        #2
        for (i = data2[6:0]; i < 8; i = i + 1) begin
            if (data2[7]) begin                     // If the shift is right
                lshift[i - data2[6:0]] = data1[i];  // Logical right shift
                ashift[i - data2[6:0]] = data1[i];  // Arithmetic right shift
            end
            else
                lshift[i] = data1[i - data2[6:0]];  // Logical left shift
        end
    end
endmodule

/*
    ROTATE module implements the bitwise ROTATE functional unit of the ALU. It performs both bitwise left and right rotation on data1 by an amount of data2 and outputs 
    the result.
*/
module ROTATE (data1, data2, result);
    // Port declarations
    input [7:0] data1, data2;
    output reg [7:0] result;

    integer i;                                                      // Counter used for the loop

    always @ * begin
        #2
        for (i = 0; i < 8; i = i + 1)
            if (data2[7])                                            // If the rotation is right
                result[i] = data1[(i + data2[6:0]) % 8];             // Right rotation
            else
                result[7 - i] = data1[(7 - i - data2[6:0]) % 8];     // Left rotation
    end
endmodule

/*
    ALU module implements Arithmetic Logic Unit for the 8-bit single cycle CPU which provides four different functional units to support the following instructions:
    add, sub, and, or mov and loadi.

    The port interfaces which the ALU module provides are as follows:
        DATA1 (Input Port, 8 bits wide)             :   For the first operand to do the arithmetic with (OPERAND1)
        DATA2 (Input Port, 8 bits wide)             :   For the second operand to do the arithmetic with (OPERAND2)
        SELECT (Control Input Port, 3 bits wide)    :   Control signal (ALU_OP) to select the particular functional unit to perform the requested arithmetic
        ZERO (Output Port, 1 bit wide)              :   Control signal FLAG to indicate that the signals at data1 and data2 ports are the same or not
        RESULT (Output Port, 8 bits wide)           :   The output (ALU_RESULT) after performing the particular arithmetic on OPERAND1 and OPERAND2
    
    When either the signals at data1 or data2 ports change, all the functional units will be triggered since they're continously driven by those signals and their
    results will be available in the 'all' bus. The appropriate signal is forwarded as the ALU_RESULT signal into the result port according to the MUX select control
    signal.
*/
module alu (data1, data2, result, zero, select);
    // Port declarations
    input [7:0] data1, data2;
    input [2:0] select;
    output [7:0] result;
    output zero;

    // Four 8-bit register vectors to store the result of each function
    wire [7:0] all[7:0];

    // Instantiate each functional module and connect the signals to the corresponding ports
    FORWARD f0 (data2, all[0]);
    ADD f1 (data1, data2, all[1]);
    AND f2 (data1, data2, all[2]);
    OR f3 (data1, data2, all[3]);
    MULT f4 (data1, data2, all[4]);
    SHIFT f5 (data1, data2, all[5], all[6]);
    ROTATE f7 (data1, data2, all[7]);

    // Behavioral model for the MUX. All the inputs to the MUX are stored in the vector 'all' and the select control signal is used as the index to access them.
    assign result = all[select];

    assign zero = (result == 8'd0);
endmodule

// ************************************************************************************************
//                                  END OF THE ALU MODULE
// ************************************************************************************************


// ************************************************************************************************
//                                8x8 REGISTER_FILE MODULE
// ************************************************************************************************

/*
    reg_file implements the 8x8 register file which stores the output values generated by the ALU and supplies the ALU's inputs with operands.
*/
module reg_file (in, out1, out2, in_address, out1_address, out2_address, write, clk, reset, busywait);
    // Port declarations
    input [7:0] in;
    input [2:0] in_address, out1_address, out2_address;
    input write, clk, reset, busywait;
    output [7:0] out1, out2;

    // 8-bit register vector of size 8
    reg signed [7:0] register_file[7:0];

    integer i;  // Used as the counter when resetting the register file
    always @ (posedge clk) begin    // Synchronous writing and resetting
        // Write the data in the port 'in' to the register specified by the in_address when write port is at high, reset port is at low
        if (write && !reset && !busywait)
            #1 register_file[in_address] = in;
        // If reset port is at high, iterate over the register file and write 0
        if (reset && !busywait) begin
            #1
            for (i = 0; i < 8; i = i + 1)
                register_file[i] = 0;
        end
    end

    // Display the content inside the register file (each register from r0 upto r7) when any one of the port signals change
    always @ *
        $display("%d %d %d %d %d %d %d %d %d", $time, register_file[0], register_file[1], register_file[2], register_file[3], register_file[4], register_file[5], register_file[6], register_file[7]);

    initial begin
        $dumpfile("cpu_ext.vcd");
        $dumpvars(0, register_file[0], register_file[1], register_file[2], register_file[3], register_file[4], register_file[5], register_file[6], register_file[7]);
    end

    // out1 and out2 is continuously driven by the value of the register specified by the out1_address and out2_address respectively
    assign #2 out1 = register_file[out1_address];
    assign #2 out2 = register_file[out2_address];
endmodule

// ************************************************************************************************
//                              END OF THE REGISTER_FILE MODULE
// ************************************************************************************************


// ************************************************************************************************
//                                      CPU MODULE
// ************************************************************************************************

module control_unit (
    instruction, 
    clk, 
    reset, 
    source_reg1, 
    source_reg2, 
    write_enable, 
    dest_reg, 
    immediate, 
    op_code, 
    twos_comp, 
    imm, 
    zero, 
    branch_jump_adder,
    read_mem,
    write_mem,
    busywait,
    data_select);

    // Port declarations
    input [31:0] instruction;                           // The current instruction which is fetched from the instruction memory
    input clk, reset, zero, busywait;
    // source_reg1:         Address of the first source register of the instruction
    // source_reg2:         Address of the second source register of the intruction
    // dest_reg:            Address of the destination register of the instruction
    // op_code:             Control signal to be sent to the ALU according to the OP-CODE of the given instruction
    output reg [2:0] source_reg1, source_reg2, dest_reg, op_code;
    // The immediate value signal to be directed to the MUX which selects whether the source2 is an immediate value or an address of a register according to the OPCODE
    output reg [7:0] immediate;
    // To drive the program counter
    //output reg [31:0] pc_target;
    // write_enable:        Control signal to be sent to the register_file to enable or disable writing into the register file
    // twos_comp:           The control signal for the MUX which selects whether or not to convert the signal at reg_out2 port into 2's complement
    // imm:                 The control signal for the MUX which selects whether the signal to be sent to the operand2 port in the ALU is an immediate value or an address
    // of a register.
    // branch_jump_adder:   Control signal for the PC Adder for branch / jump instructions
    // read_mem:            The read control signal for the data_memory
    // write_mem:           The write control signal for the read_memory
    // data_select:         The control signal for the MUX which selects the data input signal for the register_file
    output reg write_enable, twos_comp, imm, branch_jump_adder, read_mem, write_mem, data_select;

    // The encoded op-codes
    parameter op_loadi = 8'd0;
    parameter op_mov = 8'd1;
    parameter op_add = 8'd2;
    parameter op_sub = 8'd3;
    parameter op_and = 8'd4;
    parameter op_or = 8'd5;
    parameter op_j = 8'd6;
    parameter op_beq = 8'd7;
    parameter op_bne = 8'd8;
    parameter op_mult = 8'd9;
    parameter op_sll = 8'd10;
    parameter op_srl = 8'd11;
    parameter op_sla = 8'd12;
    parameter op_sra = 8'd13;
    parameter op_rol = 8'd14;
    parameter op_ror = 8'd15;
    parameter op_lwd = 8'd16;
    parameter op_lwi = 8'd17;
    parameter op_swd = 8'd18;
    parameter op_swi = 8'd19;

    // Control signals for the ALU
    parameter alu_forward = 3'd0;
    parameter alu_add = 3'd1;
    parameter alu_and = 3'd2;
    parameter alu_or = 3'd3;
    parameter alu_mult = 3'd4;
    parameter alu_lshift = 3'd5;
    parameter alu_ashift = 3'd6;
    parameter alu_rotate = 3'd7;

    /* DECODING THE CURRENT INSTRUCTION */
    // Processing the instruction is sensitive to the change in the instruction. Because an instruction should be proccessed just after it has been fetched completely.
    always @ instruction begin
        write_enable = 0;
        branch_jump_adder = 0;
        write_mem = 0;
        read_mem = 0;
        // Implementation of the MUX which decodes the current instruction and sends the appropriate control signals in order to execute the instruction.
        case (instruction[31:24])                           // Bits 31 to 26 carries the OP-CODE of the instruction
            // If the OP-CODE is 'loadi'
            // SYNTAX:          loadi      RD     <unused>    IMM
            // MACHINE CODE:  00000000  XXXXXXXX  00000000  XXXXXXXX
            op_loadi: begin
                write_enable = 1;                           // loadi instruction requires writting the given immediate value into the specified register RD
                #1
                immediate = instruction[7:0];               // loadi instruction carries an immediate value at the least significant byte
                dest_reg = instruction[18:16];              // The address of the destination register is in the bits 18 to 16 (because a register address is 3-bits wide)
                op_code = alu_forward;                      // Send the relevant alu_op signal to the ALU (FORWARD control signal)
                imm = 1;                                    // Send the imm control signal to the corresponding MUX to select the signal at operand2 as an immediate value
                twos_comp = 0;                              // Send the twos_comp signal as LOW to the corresponding MUX because the signal doesn't need to be converted
                data_select = 0;                            // The data in signal for the register_file should be the alu_result
            end
            // If the OP-CODE is 'mov'
            // SYNTAX:          mov        RD     <unused>    RS2
            // MACHINE CODE:  00000001  XXXXXXXX  00000000  XXXXXXXX
            op_mov: begin
                write_enable = 1;                           // Set write_enable to HIGH in order to write into the register specified by the bits 18:16
                #1
                source_reg2 = instruction[2:0];             // Set the address at the read_reg2 port in the register file to the bits 2:0
                dest_reg = instruction[18:16];              // Set the address of the destination register
                op_code = alu_forward;                      // Set the alu_op control signal to FORWARD
                imm = 0;                                    // Set the imm control signal to LOW because the source2 contains an address of a register
                twos_comp = 0;                              // Set the twos_comp signal to LOW
                data_select = 0;                            // The data in signal for the register_file should be the alu_result
            end
            // If the OP-CODE is 'add'
            // SYNTAX:          add        RD        RS1      RS2
            // MACHINE CODE:  00000010  XXXXXXXX  XXXXXXXX  XXXXXXXX
            op_add: begin
                write_enable = 1;                           // Set write_enable to HIGH in order to write into the register specified by the bits 18:16
                #1
                source_reg1 = instruction[10:8];            // Set the address at the read_reg1 port in the register file to the bits 10:8
                source_reg2 = instruction[2:0];             // Set the address at the read_reg2 port in the register file to the bits 2:0        
                dest_reg = instruction[18:16];              // Set the address of the destination register
                op_code = alu_add;                          // The operands at the ALU should be handled by the ADD functional unit
                imm = 0;                                    // Set the imm control signal to LOW because the source2 contains an address of a register
                twos_comp = 0;                              // Set the twos_comp signal to LOW
                data_select = 0;                            // The data in signal for the register_file should be the alu_result
            end
            // If the OP-CODE is 'sub'
            // SYNTAX:          sub        RD        RS1      RS2
            // MACHINE CODE:  00000011  XXXXXXXX  XXXXXXXX  XXXXXXXX
            op_sub: begin
                write_enable = 1;                           // Set write_enable to HIGH in order to write into the register specified by the bits 18:16
                #1
                source_reg1 = instruction[10:8];            // Set the address at the read_reg1 port in the register file to the bits 10:8
                source_reg2 = instruction[2:0];             // Set the address at the read_reg2 port in the register file to the bits 2:0
                dest_reg = instruction[18:16];              // Set the address of the destination register
                op_code = alu_add;                          // The operands at the ALU should be handled by the ADD functional unit
                imm = 0;                                    // Set the imm control signal to LOW because the source2 contains an address of a register
                twos_comp = 1;                              // Operand2 eeds to be converted to the twos complement representation in order to subtract
                data_select = 0;                            // The data in signal for the register_file should be the alu_result
            end
            // If the OP-CODE is 'and'
            // SYNTAX:          and        RD        RS1      RS2
            // MACHINE CODE:  00000100  XXXXXXXX  XXXXXXXX  XXXXXXXX
            op_and: begin
                write_enable = 1;                           // Set write_enable to HIGH in order to write into the register specified by the bits 18:16
                #1
                source_reg1 = instruction[10:8];            // Set the address at the read_reg1 port in the register file to the bits 10:8
                source_reg2 = instruction[2:0];             // Set the address at the read_reg2 port in the register file to the bits 2:0
                dest_reg = instruction[18:16];              // Set the address of the destination register
                op_code = alu_and;                          // The operands at the ALU should be handled by the AND functional unit
                imm = 0;                                    // Set the imm control signal to LOW because the source2 contains an address of a register
                twos_comp = 0;                              // Set the twos_comp signal to LOW
                data_select = 0;                            // The data in signal for the register_file should be the alu_result
            end
            // If the OP-CODE is 'or'
            // SYNTAX:           or        RD        RS1      RS2
            // MACHINE CODE:  00000101  XXXXXXXX  XXXXXXXX  XXXXXXXX
            op_or: begin
                #1
                write_enable = 1;                           // Set write_enable to HIGH in order to write into the register specified by the bits 18:16
                source_reg1 = instruction[10:8];            // Set the address at the read_reg1 port in the register file to the bits 10:8
                source_reg2 = instruction[2:0];             // Set the address at the read_reg2 port in the register file to the bits 2:0
                dest_reg = instruction[18:16];              // Set the address of the destination register
                op_code = alu_or;                           // The operands at the ALU should be handled by the OR functional unit
                imm = 0;                                    // Set the imm control signal to LOW because the source2 contains an address of a register
                twos_comp = 0;                              // Set the twos_comp signal to LOW
                data_select = 0;                            // The data in signal for the register_file should be the alu_result
            end
            // If the OP-CODE is 'j'
            // SYNTAX:            j      offset    <----unused---->    
            // MACHINE CODE:  00000110  XXXXXXXX  XXXXXXXX  XXXXXXXX
            op_j: begin
                branch_jump_adder = 1;
                #1 data_select = 0;
            end
            // If the OP-CODE is 'beq'
            // SYNTAX:          beq      offset      RS1      RS2    
            // MACHINE CODE:  00000111  XXXXXXXX  XXXXXXXX  XXXXXXXX
            op_beq: begin
                branch_jump_adder = 1; 
                #1
                source_reg1 = instruction[10:8];            // Set the address at the read_reg1 port in the register file to the bits 10:8
                source_reg2 = instruction[2:0];             // Set the address at the read_reg2 port in the register file to the bits 2:0                     
                op_code = alu_add;                          // The operands at the ALU should be handled by the ADD functional unit
                imm = 0;                                    // Set the imm control signal to LOW because the source2 contains an address of a register
                twos_comp = 1;
                data_select = 0;                            // The data in signal for the register_file should be the alu_result
            end
            // If the OP-CODE is 'bne'
            // SYNTAX:          bne      offset     RS1       RS2    
            // MACHINE CODE:  00001000  XXXXXXXX  XXXXXXXX  XXXXXXXX
            op_bne: begin
                branch_jump_adder = 1;
                #1
                source_reg1 = instruction[10:8];            // Set the address at the read_reg1 port in the register file to the bits 10:8
                source_reg2 = instruction[2:0];             // Set the address at the read_reg2 port in the register file to the bits 2:0
                op_code = alu_add;                          // The operands at the ALU should be handled by the ADD functional unit
                imm = 0;                                    // Set the imm control signal to LOW because the source2 contains an address of a register
                twos_comp = 1;                              // Set the twos_comp signal to HIGH because the operation should be a subtraction
                data_select = 0;
            end
            // If the OP-CODE is 'mult'
            // SYNTAX:          mult       RD       RS1       RS2    
            // MACHINE CODE:  00001001  XXXXXXXX  XXXXXXXX  XXXXXXXX
            op_mult: begin
                write_enable = 1;                           // Set write_enable to HIGH in order to write into the register specified by the bits 18:16
                #1
                source_reg1 = instruction[10:8];            // Set the address at the read_reg1 port in the register file to the bits 10:8
                source_reg2 = instruction[2:0];             // Set the address at the read_reg2 port in the register file to the bits 2:0
                dest_reg = instruction[18:16];              // Set the address of the destination register
                op_code = alu_mult;                         // The operands at the ALU should be handled by the MULT functional unit
                imm = 0;                                    // Set the imm control signal to LOW because the source2 contains an address of a register
                twos_comp = 0;                              // Set the twos_comp signal to LOW
                data_select = 0;
            end
            // If the OP-CODE is 'sll'
            // SYNTAX:          sll        RD        RS1      IMM    
            // MACHINE CODE:  00001010  XXXXXXXX  XXXXXXXX  XXXXXXXX
            op_sll: begin
                write_enable = 1;                           // Set write_enable to HIGH in order to write into the register specified by the bits 18:16
                #1
                source_reg1 = instruction[10:8];            // Set the address at the read_reg1 port in the register file to the bits 10:8
                immediate = {1'b0, instruction[6:0]};       // Set the immediate value, set msb to 0 to indicate its a left shift
                dest_reg = instruction[18:16];              // Set the address of the destination register
                op_code = alu_lshift;                       // The operands at the ALU should be handled by the SHIFT functional unit
                imm = 1;                                    // Send the imm control signal to the corresponding MUX to select the signal at operand2 as an immediate value
                twos_comp = 0;                              // Set the twos_comp signal to LOW
                data_select = 0;
            end
            // If the OP-CODE is 'srl'
            // SYNTAX:          srl        RD        RS1      IMM    
            // MACHINE CODE:  00001011  XXXXXXXX  XXXXXXXX  XXXXXXXX
            op_srl: begin
                write_enable = 1;                           // Set write_enable to HIGH in order to write into the register specified by the bits 18:16
                #1
                source_reg1 = instruction[10:8];            // Set the address at the read_reg1 port in the register file to the bits 10:8
                immediate = {1'b1, instruction[6:0]};       // Set the immediate value, set msb to 1 to indicate its a right shift
                dest_reg = instruction[18:16];              // Set the address of the destination register
                op_code = alu_lshift;                       // The operands at the ALU should be handled by the SHIFT functional unit
                imm = 1;                                    // Send the imm control signal to the corresponding MUX to select the signal at operand2 as an immediate value
                twos_comp = 0;                              // Set the twos_comp signal to LOW
                data_select = 0;
            end
            // If the OP-CODE is 'sla'
            // SYNTAX:          sla        RD        RS1      IMM    
            // MACHINE CODE:  00001100  XXXXXXXX  XXXXXXXX  XXXXXXXX
            op_sla: begin
                write_enable = 1;                           // Set write_enable to HIGH in order to write into the register specified by the bits 18:16
                #1
                source_reg1 = instruction[10:8];            // Set the address at the read_reg1 port in the register file to the bits 10:8
                immediate = {1'b0, instruction[6:0]};       // Set the immediate value, set msb to 0 to indicate its a left shift
                dest_reg = instruction[18:16];              // Set the address of the destination register
                // Arithmetic left shift is the same as Logical left shift
                op_code = alu_lshift;                       // The operands at the ALU should be handled by the SHIFT functional unit
                imm = 1;                                    // Send the imm control signal to the corresponding MUX to select the signal at operand2 as an immediate value
                twos_comp = 0;                              // Set the twos_comp signal to LOW
                data_select = 0;
            end
            // If the OP-CODE is 'sra'
            // SYNTAX:          sra        RD        RS1      IMM    
            // MACHINE CODE:  00001101  XXXXXXXX  XXXXXXXX  XXXXXXXX
            op_sra: begin
                write_enable = 1;                           // Set write_enable to HIGH in order to write into the register specified by the bits 18:16
                #1
                source_reg1 = instruction[10:8];            // Set the address at the read_reg1 port in the register file to the bits 10:8
                immediate = {1'b1, instruction[6:0]};       // Set the immediate value, set msb to 1 to indicate its a right shift
                dest_reg = instruction[18:16];              // Set the address of the destination register
                op_code = alu_ashift;                       // The operands at the ALU should be handled by the SHIFT functional unit
                imm = 1;                                    // Send the imm control signal to the corresponding MUX to select the signal at operand2 as an immediate value
                twos_comp = 0;                              // Set the twos_comp signal to LOW
                data_select = 0;
            end
            // If the OP-CODE is 'rol'
            // SYNTAX:          rol        RD        RS1      IMM    
            // MACHINE CODE:  00001110  XXXXXXXX  XXXXXXXX  XXXXXXXX
            op_rol: begin
                write_enable = 1;                           // Set write_enable to HIGH in order to write into the register specified by the bits 18:16
                #1
                source_reg1 = instruction[10:8];            // Set the address at the read_reg1 port in the register file to the bits 10:8
                immediate = {1'b0, instruction[6:0]};       // Set the immediate value, set msb to 0 to indicate its a left rotate
                dest_reg = instruction[18:16];              // Set the address of the destination register
                op_code = alu_rotate;                       // The operands at the ALU should be handled by the ROR functional unit
                imm = 1;                                    // Send the imm control signal to the corresponding MUX to select the signal at operand2 as an immediate value
                twos_comp = 0;                              // Set the twos_comp signal to LOW
                data_select = 0;
            end
            // If the OP-CODE is 'ror'
            // SYNTAX:          ror        RD        RS1      IMM    
            // MACHINE CODE:  00001111  XXXXXXXX  XXXXXXXX  XXXXXXXX
            op_ror: begin
                write_enable = 1;                           // Set write_enable to HIGH in order to write into the register specified by the bits 18:16
                #1
                source_reg1 = instruction[10:8];            // Set the address at the read_reg1 port in the register file to the bits 10:8
                immediate = {1'b1, instruction[6:0]};       // Set the immediate value, set msb to 1 to indicate its a right rotate
                dest_reg = instruction[18:16];              // Set the address of the destination register
                op_code = alu_rotate;                       // The operands at the ALU should be handled by the ROR functional unit
                imm = 1;                                    // Send the imm control signal to the corresponding MUX to select the signal at operand2 as an immediate value
                twos_comp = 0;                              // Set the twos_comp signal to LOW
                data_select = 0;
            end
            // If the OP-CODE is 'lwd'
            // SYNTAX:           lwd       RD     <unused>     RS  
            // MACHINE CODE:  00010000  XXXXXXXX  XXXXXXXX  XXXXXXXX
            op_lwd: begin
                write_enable = 1;
                source_reg2 = instruction[2:0];
                dest_reg = instruction[18:16];
                #2
                imm = 0;
                twos_comp = 0;
                op_code = alu_forward;
                read_mem = 1;
                data_select = 1;
            end
            // If the OP-CODE is 'lwi'
            // SYNTAX:           lwi       RD     <unused>    IMM    
            // MACHINE CODE:  00010001  XXXXXXXX  XXXXXXXX  XXXXXXXX
            op_lwi: begin
                write_enable = 1;
                #1
                immediate = instruction[7:0];
                dest_reg = instruction[18:16];
                twos_comp = 0;
                op_code = alu_forward;
                imm = 1;
                data_select = 1;
                read_mem = 1;
            end
            // If the OP-CODE is 'swd'
            // SYNTAX:           swd    <unused>     RT        RS   
            // MACHINE CODE:  00010010  XXXXXXXX  XXXXXXXX  XXXXXXXX
            op_swd: begin
                source_reg1 = instruction[10:8];
                source_reg2 = instruction[2:0];
                #2
                imm = 0;
                twos_comp = 0;
                op_code = alu_forward;
                write_mem = 1;
                data_select = 0;
            end
            // If the OP-CODE is 'swi'
            // SYNTAX:           swi    <unused>     RT       IMM   
            // MACHINE CODE:  00010011  XXXXXXXX  XXXXXXXX  XXXXXXXX
            op_swi: begin
                #1
                immediate = instruction[7:0];
                source_reg1 = instruction[10:8];
                imm = 1;
                twos_comp = 0;
                op_code = alu_forward;
                write_mem = 1;
                data_select = 0;
            end
        endcase
    end

    always @ (negedge busywait) begin
        write_mem = 0;
        read_mem = 0;
    end
endmodule

module cpu (pc, instruction, clk, reset, read_mem, write_mem, address_mem, readdata, busywait, writedata);
    // Port declarations
    input clk, reset, busywait;
    input [7:0] readdata;
    input [31:0] instruction;
    output reg [31:0] pc, pc_target;
    output  read_mem,write_mem;
    output [7:0] address_mem, writedata;

    reg signed [31:0] offset;
    reg [31:0] pc_update;
    reg [7:0] operand2, twos_complement;        // Signal at the operand2 in ALU and the register to save the twos_complement
    wire [2:0] alu_op;                          // Control signals for the select port in ALU
    wire [7:0] alu_result;                      // Output signal from the ALU
    wire [2:0] write_reg, read_reg1, read_reg2; // 3-bit address signals to be fed into in_address, out1_address and out2_address ports in reg_file
    wire write_enable, clk, reset;              // Write-enable, clock and reset signals
    wire [7:0] reg_out1, reg_out2;              // To store output signals from out1 and out2 ports in reg_file
    wire twos_comp, imm, zero;                  // Control signals to select the operand2 and the ZERO signal for the beq instruction
    wire [7:0] immediate;                       // To carry the immediate value provided by the instruction
    wire branch_jump_adder;
    wire data_select;
    reg [7:0] data;

    parameter op_j = 8'd6;
    parameter op_beq = 8'd7;
    parameter op_bne = 8'd8;

    // Instantiating the alu module
    alu _alu (
        .data1(reg_out1),                       // Connect reg_out1 8-bit signal bus to the data1 port in ALU
        .data2(operand2),                       // Connect operand2 8-bit signal bus to the data2 port in ALU
        .result(alu_result),                    // Connect the alu_result 8-bit signal bus to the output port result in ALU
        .zero(zero),                            // Connect the corresponding wire to the ZERO port in ALU
        .select(alu_op));                       // Connect alu_op 3-bit control signal bus to the select port in ALU

    // Instantiating the reg_file module
    reg_file _reg (
        .in(data),                              // Connect the alu_result 8-bit bus to the in port in reg_file
        .out1(reg_out1),                        // Connect reg_out1 to out1 port in re_file
        .out2(reg_out2),                        // Connect reg_out2 to out2 port in reg_file
        .in_address(write_reg),                 // Connect write_reg 3-bit bus to in_address port in reg_file
        .out1_address(read_reg1),               // Connect read_reg1 3-bit bus to out1_address port in reg_file
        .out2_address(read_reg2),               // Connect read_reg2 3-bit bus to out2_address port in reg_file
        .write(write_enable),                   // Connect write_enable signal to write port in re_file
        .clk(clk),                              // Connect clock signal to clk port in reg_file
        .reset(reset),
        .busywait(busywait));                   // Connect reset signal to the corresponding port in reg_file

    // Instantiating the control_unit module
    control_unit _control (
        .instruction(instruction),              // Connect the instruction bus
        .clk(clk),                              // Connect the clock signal
        .reset(reset),                          // Connect the reset signal
        .source_reg1(read_reg1),                // Connect the read_reg1 signal to the source_reg1 port in the control_unit
        .source_reg2(read_reg2),                // Connect the read_reg2 signal to the source_reg2 port in the control_unit
        .write_enable(write_enable),            // Connect the write_enable signal
        .dest_reg(write_reg),                   // Connect the write_reg address signal to the dest_reg port in the control unit
        .immediate(immediate),                  // Connect the immediate signal
        .op_code(alu_op),                       // Connect the alu_op control signal to the op_code port in the control_unit
        .twos_comp(twos_comp),                  // Connect the twos_comp control signal
        .imm(imm),                              // Connect the imm control signal
        .zero(zero),
        .branch_jump_adder(branch_jump_adder),
        .read_mem(read_mem),
        .write_mem(write_mem),
        .busywait(busywait),
        .data_select(data_select));                           
    
    always @ * begin
        if (twos_comp)
            #2 twos_complement = (reg_out2 ^ 8'd255) + 1;      // XOR with all 1's to invert the bits
        else
            twos_complement = reg_out2;
        if (imm)
            operand2 = immediate;
        else
            operand2 = twos_complement;
    end

    // Program Counter
    always @ (posedge clk) begin
        // If the reset signal is HIGH at the positive clock edge, set the PC to 0
        if (!busywait) begin
            if (reset)
                pc_update = 0;
            // Updating the PC
            #1
            // If the previous instruction was a branch and the ZERO flag has the relevant signal, update the next pc address with the previously calculated pc_target
            if (instruction[31:24] == op_j || (instruction[31:24] == op_beq && zero) || (instruction[31:24] == op_bne && !zero))
                pc_update = pc_target;
            pc = pc_update;
            // PC Adder
            #1 pc_update = pc_update + 4;
        end
    end

    always @ branch_jump_adder begin
        if (branch_jump_adder) begin
            offset = {{22{instruction[23]}}, instruction[23:16], {2{1'b0}}};
            #2 pc_target = pc_update + offset;                              
        end
    end

    assign address_mem = alu_result;
    assign writedata = reg_out1;

    always @ (data_select, readdata, alu_result)
        data = data_select ? readdata : alu_result;

    // initial begin
    //     $dumpfile("cpu_ext.vcd");
    //     $dumpvars(0, cpu);
    // end
endmodule

// ************************************************************************************************
//                                  END OF THE CPU MODULE
// ************************************************************************************************


// ************************************************************************************************
//                                      DATA MEMORY
// ************************************************************************************************

/*
    Memory Address:
    <-TAG BITS-><-CACHE ID-><OFFSET->
    | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
*/
module cache_controller (clock, reset, read, mem_read, write, mem_write, address, mem_address, writedata, mem_writedata, readdata, mem_readdata, busywait, mem_busywait);
    // Port declaration
    input clock, reset, read, write, mem_busywait;
    input [7:0] address, writedata;
    input [31:0] mem_readdata;
    output reg busywait, mem_read, mem_write;
    output reg [5:0] mem_address;
    output reg [7:0] readdata;
    output reg [31:0] mem_writedata;

    // hit: Temporary signal to store whether the cache access is a hit or a miss
    // comparator: Temporary signal from the comparator which compares the tag bits of the indexed cache Line and requested address by the CPU
    // writeaccess: The temporary signal to indicate that the writedata should be written into the cache
    reg hit, comparator, writeaccess;
    // cache: vector of 8, 37-bit long vectors for the cache memory
    // 3 most significant bits store the tag for the particular Line
    // 32nd and 33rd bits store the dirty bit and the valid bit for the particular Line respectively
    // line: The current indexed cache line bus
    reg [36:0] cache [7:0], line;
    // The data word in the selected cache line corresponding to the block offset
    reg [7:0] readword;

    // cache_index: The bus which carries the index of the cache line
    // tag_bits: The bus which carries the tag bits of the CPU request
    wire [2:0] cache_index, tag_bits;
    // block offset * 8
    wire [4:0] word_index;

    // Splitting the address
    assign word_index = {address[1:0], 3'b000};                 // Multiplication by 8 to get the starting index of the data word
    assign cache_index = address[4:2];
    assign tag_bits = address[7:5];

    // Generating the busywait signal
    always @ (posedge read, posedge write)
        busywait = read || write;

    always @ (line, tag_bits) begin
        // The comparator which compares the requested tag bits with the tag bits of the indexed cache line
        #0.9 comparator = tag_bits == line[36:34];
        // The hit / miss signal
        hit = line[33] && comparator;
    end

    always @ (posedge busywait, negedge mem_busywait) begin
        if (busywait) begin
            #1 // Wait for the address to be settled
            // Extracting the cache line by indexing the cache memory
            #1 line = cache[cache_index];
            // MUX to read select the requested data word from the cache memory by the requested block offset (address[1:0]) as the control signal
            #1 readword = line[word_index +: 8];
        end

        // If the request is not a data memory fetch
        if (!mem_read) begin
            // If the cache access is a hit
            if (hit) begin
                // If the cache access is a read-hit
                if (read) begin
                    readdata = readword;                        // Direct the readword bus to the readdata port
                    busywait = 0;                               // No need to stall the CPU. Set the busywait signal to low
                end
                // If the cache access is a write-hit
                if (write) begin
                    writeaccess = 1;                            // The requested data has to be written to the cache memory on the next positive clock edge
                    busywait = 0;                               // No need to stall the CPU. Set the busywait signal to low
                end
            end
            // If the cache access is a miss and the dirty bit is high
            else if (line[32]) begin
                mem_write = 1;                                  // A write-back has to be done prior to the data memory fetch
                mem_address = {line[36:34], cache_index};       // Direct the corresponding memory address of the block to the mem_address port
                mem_writedata = line[31:0];                     // Direct the cache data block to the mem_writedata port
            end
            // If the cache access is a miss and the dirty bit is low
            else begin
                mem_read = 1;                                   // The missing block has to be fetched from the data memory
                mem_address = address[7:2];                     // Set the last 6 bits of the requested address to the mem_address port
            end
        end
    end

    // Data memory read, write-back and writting to the cache
    always @ (posedge clock) begin
        if (writeaccess) begin
            // DEMUX to select the word in the cache line to be written into by the requested block offset (address[1:0]) as the control signal
            #1 cache[cache_index][word_index +: 8] = writedata;
            // Mark the line as dirty
            cache[cache_index][32] = 1;
            // Set the writeaccess signal to low
            writeaccess = 0;
        end
    end

    // After a writeback or a memory read has been completed
    always @ (negedge mem_busywait) begin
        // If the previous action was a writeback
        if (mem_write) begin
            mem_write = 0;
            // If the current access is a miss with dirty bit high, the next action should be a data memory fetch
            if ((read || write) && !hit && line[32]) begin
                mem_read = 1;
                mem_address = address[7:2];
            end
        end
        // If the previous action was a data memory fetch
        else if (mem_read) begin
            mem_read = 0;
            // Write the fetched data into the particular indexed line of the cache
            #1 cache[cache_index] = {tag_bits, 2'b10, mem_readdata};
        end
    end

    integer i;

    always @ (posedge clock) begin
        if (reset) begin
            // Set the cache lines to be invalid
            for (i = 0; i < 8; i = i + 1)
                cache[i][33] = 0;
            busywait = 0;
            mem_read = 0;
            mem_write = 0;
            writeaccess = 0;
        end
    end

    // initial begin
    //     $dumpfile("cpu_ext.vcd");
    //     $dumpvars(0, cache[0], cache[1], cache[2], cache[3], cache[4], cache[5], cache[6], cache[7]);
    // end
endmodule


module data_memory (clock, reset, read, write, address, writedata, readdata, busywait);
    input clock, reset, read, write;
    input [5:0] address;
    input [31:0] writedata;
    output reg [31:0] readdata;
    output reg busywait;

    //Declare memory array 256x8-bits 
    reg [7:0] memory_array [255:0];
    //Detecting an incoming memory access
    reg readaccess, writeaccess;

    always @ (read, write) begin
        busywait = read || write;
        readaccess = read && !write;
        writeaccess = !read && write;
    end

    //Reading & writing
    always @ (posedge clock) begin
        if (readaccess) begin
            readdata[7:0]   = #40 memory_array[{address, 2'b00}];
            readdata[15:8]  = #40 memory_array[{address, 2'b01}];
            readdata[23:16] = #40 memory_array[{address, 2'b10}];
            readdata[31:24] = #40 memory_array[{address, 2'b11}];
            busywait = 0;
            readaccess = 0;
        end
        if (writeaccess) begin
            memory_array[{address, 2'b00}] = #40 writedata[7:0];
            memory_array[{address, 2'b01}] = #40 writedata[15:8];
            memory_array[{address, 2'b10}] = #40 writedata[23:16];
            memory_array[{address, 2'b11}] = #40 writedata[31:24];
            busywait = 0;
            writeaccess = 0;
        end
    end

    integer i;

    //Reset memory
    always @ (posedge reset) begin
        if (reset) begin
            for (i = 0; i < 256; i = i + 1)
                memory_array[i] = 0;
            busywait = 0;
            readaccess = 0;
            writeaccess = 0;
        end
    end

    // initial begin
    //     $dumpfile("cpu_ext.vcd");
    //     $dumpvars(0, memory_array[0], memory_array[1], memory_array[2], memory_array[9], memory_array[32]);
    // end

    // always @ *
    //     $display("%d %d %d %d %d", $time, memory_array[0], memory_array[1], memory_array[2], memory_array[3]);
endmodule

// ************************************************************************************************
//                                   END OF THE DATA MEMORY
// ************************************************************************************************


// ************************************************************************************************
//                                      INSTRUCTION MEMORY
// ************************************************************************************************

module instruction_cache_controller(clock, pc, reset, imem_busywait, readinst, busywait, read, instruction, address);
    // Port declaration
    input imem_busywait, reset, clock;
    input [31:0] pc;
    input [127:0] readinst;
    output reg busywait, read;
    output reg [31:0] instruction;
    output reg [5:0] address;

    // hit: Temporary signal to store whether the cache access is a hit or a miss
    // comparator: Temporary signal from the comparator which compares the tag bits of the indexed cache Line and the PC address
    // readaccess: Temporary signal to read from the cache when a instruction memory fetch is completed
    reg hit, comparator, readaccess;
    // cache: vector of 8, 132-bit long vectors for the cache memory
    // 4 most significant bits store the valid bit and the tag bits for the particular Line
    // line: The current indexed cache line bus
    reg [131:0] cache [7:0], line;

    // cache_index: The bus which carries the index of the cache line
    // tag_bits: The bus which carries the tag bits of the PC address
    wire [2:0] tag_bits, cache_index;
    // Starting index of the word to be selected from the block: cache_index * 8
    wire [6:0] word_index;

    // Splitting the address
    assign word_index = {pc[3:0], 3'b000};      // Multiply the block offset by 8 to get the start index of the word to be selected
    assign cache_index = pc[6:4];
    assign tag_bits = pc[9:7];

    always @ (pc, posedge readaccess) begin
        #1 line = cache[cache_index];
    end
        

    always @ (line, tag_bits, word_index) begin
        #1 comparator = tag_bits == line[130:128];
        hit = comparator && line[131];
        read = !hit;

        // Read the instruction from the cache memory
        if (hit)
            #1 instruction = line[word_index +: 32];
    end

    always @ (posedge read) begin
        address = {tag_bits, cache_index};
        busywait = 1;
    end

    always @ (negedge imem_busywait) begin
        readaccess = 1;
        // Fetch the instruction from the memory
        #1 cache[cache_index] = {1'b1, tag_bits, readinst};
    end

    integer i;

    always @ (posedge clock) begin
        // Set the cache lines to be invalid
        if (reset) begin
            for (i = 0; i < 8; i = i + 1)
                cache[i][131] = 0;
            busywait = 0;
        end
        if (readaccess) begin
            readaccess = 0;
            busywait = 0;
        end
    end
endmodule

module instruction_memory(clock, read, address, readinst, busywait);
    input clock, read;
    input [5:0] address;
    output reg [127:0] readinst;
    output reg busywait;

    reg readaccess;
    //Declare memory array 1024x8-bits 
    reg [7:0] memory_array [1023:0];

    //Initialize instruction memory
    initial begin
        busywait = 0;
        readaccess = 0;
        $readmemb("instr_mem.mem", memory_array);	
    end

    //Detecting an incoming memory access
    always @ read begin
        busywait = read;
        readaccess = read;
    end

    //Reading
    always @ (posedge clock) begin
        if(readaccess) begin
            readinst[7:0]     = #40 memory_array[{address,4'b0000}];
            readinst[15:8]    = #40 memory_array[{address,4'b0001}];
            readinst[23:16]   = #40 memory_array[{address,4'b0010}];
            readinst[31:24]   = #40 memory_array[{address,4'b0011}];
            readinst[39:32]   = #40 memory_array[{address,4'b0100}];
            readinst[47:40]   = #40 memory_array[{address,4'b0101}];
            readinst[55:48]   = #40 memory_array[{address,4'b0110}];
            readinst[63:56]   = #40 memory_array[{address,4'b0111}];
            readinst[71:64]   = #40 memory_array[{address,4'b1000}];
            readinst[79:72]   = #40 memory_array[{address,4'b1001}];
            readinst[87:80]   = #40 memory_array[{address,4'b1010}];
            readinst[95:88]   = #40 memory_array[{address,4'b1011}];
            readinst[103:96]  = #40 memory_array[{address,4'b1100}];
            readinst[111:104] = #40 memory_array[{address,4'b1101}];
            readinst[119:112] = #40 memory_array[{address,4'b1110}];
            readinst[127:120] = #40 memory_array[{address,4'b1111}];
            busywait = 0;
            readaccess = 0;
        end
    end
endmodule

// ************************************************************************************************
//                                 END OF THE INSTRUCTION MEMORY
// ************************************************************************************************


// ************************************************************************************************
//                                  TOP LEVEL STIMULUS MODULE
// ************************************************************************************************

module stimulus;
    reg CLK, RESET;
    wire [31:0] INSTRUCTION;
    wire write, read, busywait, mem_write, mem_read, mem_busywait, imem_read, imem_busywait;
    wire [5:0] mem_address;
    wire [5:0] imem_address;
    wire [7:0] address, readdata, writedata;
    wire [31:0] PC, mem_writedata, mem_readdata;
    wire [127:0] readinst;

    // Instantiation of the cpu module
    cpu _cpu (
        .pc(PC),
        .instruction(INSTRUCTION),
        .clk(CLK),
        .reset(RESET),
        .read_mem(read),
        .write_mem(write),
        .address_mem(address),
        .readdata(readdata),
        .busywait(busywait),
        .writedata(writedata));

    // Instantiation of the instruction_cache_controller module
    instruction_cache_controller _imem_cache (
        .clock(CLK),
        .pc(PC),
        .reset(RESET),
        .imem_busywait(imem_busywait),
        .readinst(readinst),
        .busywait(busywait),
        .read(imem_read),
        .instruction(INSTRUCTION),
        .address(imem_address));

    // Instantiation of the instruction_memory module
    instruction_memory _imemory (
        .clock(CLK),
        .read(imem_read),
        .address(imem_address),
        .readinst(readinst),
        .busywait(imem_busywait));

    // Instantiation of the cache_controller module
    cache_controller _cache (
        .clock(CLK),
        .reset(RESET),
        .read(read), 
        .mem_read(mem_read), 
        .write(write), 
        .mem_write(mem_write), 
        .address(address), 
        .mem_address(mem_address), 
        .writedata(writedata), 
        .mem_writedata(mem_writedata), 
        .readdata(readdata), 
        .mem_readdata(mem_readdata), 
        .busywait(busywait), 
        .mem_busywait(mem_busywait));

    // Instantiation of the data_memory module
    data_memory _data_memory(
        .clock(CLK),
        .reset(RESET),
        .read(mem_read),
        .write(mem_write),
        .address(mem_address),
        .writedata(mem_writedata),
        .readdata(mem_readdata),
        .busywait(mem_busywait));

    initial begin
        $display("\t\t   T   r0  r1  r2  r3  r4  r5  r6  r7");
        CLK = 0;
        RESET = 1;
        #3000 $finish;
    end

    always @ (posedge CLK) begin
        #1
        if (RESET)
            RESET = 0;
    end

    always
        #4 CLK = ~CLK;

    initial
        $dumpvars(0, stimulus);
endmodule

// ************************************************************************************************
//                                  END OF STIMULUS MODULE
// ************************************************************************************************