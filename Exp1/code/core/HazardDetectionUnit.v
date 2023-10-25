`timescale 1ps/1ps

module HazardDetectionUnit(
    input clk,
    input Branch_ID, rs1use_ID, rs2use_ID,
    input[1:0] hazard_optype_ID,
    input[4:0] rd_EXE, rd_MEM, rs1_ID, rs2_ID, rs2_EXE,
    output PC_EN_IF, reg_FD_EN, reg_FD_stall, reg_FD_flush,
        reg_DE_EN, reg_DE_flush, reg_EM_EN, reg_EM_flush, reg_MW_EN,
    output wire forward_ctrl_ls,
    output[1:0] forward_ctrl_A, forward_ctrl_B
);
    /* hazard参数 */
    parameter hazard_at_ALU   =   2'b01;   // 当前阶段是否使用ALU
    parameter hazard_at_LOAD  =   2'b10;   // 当前阶段是否有load操作
    parameter hazard_at_STORE =   2'b11;   // 当前阶段是否有store操作
    

    /* 因为每个阶段的对应变量不同，�?要分阶段存储 */
    reg[1:0] hazard_optype_EXE, hazard_optype_MEM;
    
    initial begin
        hazard_optype_EXE <= 2'b00;
        hazard_optype_MEM <= 2'b00;
    end
    
    always@(posedge clk) begin
        hazard_optype_EXE <= hazard_optype_ID & {2{~reg_DE_flush}};
        hazard_optype_MEM <= hazard_optype_EXE;
    end

    /* Forwarding 条件 */
    wire rs1_forward_EX  = rs1use_ID && rs1_ID == rd_EXE && rd_EXE != 0 &&
                              hazard_optype_EXE == hazard_at_ALU;
    wire rs1_forward_EX_MEM = rs1use_ID && rs1_ID == rd_MEM && rd_MEM != 0 &&
                              hazard_optype_MEM == hazard_at_ALU;
    wire rs1_forward_MEM_WB = rs1use_ID && rs1_ID == rd_MEM && rd_MEM != 0 &&
                              hazard_optype_MEM == hazard_at_LOAD;

    wire rs2_forward_EX  = rs2use_ID && rs2_ID == rd_EXE && rd_EXE != 0 &&
                              hazard_optype_EXE == hazard_at_ALU;
    wire rs2_forward_EX_MEM = rs2use_ID && rs2_ID == rd_MEM && rd_MEM != 0 &&
                              hazard_optype_MEM == hazard_at_ALU;
    wire rs2_forward_MEM_WB = rs2use_ID && rs2_ID == rd_MEM && rd_MEM != 0 &&
                              hazard_optype_MEM == hazard_at_LOAD;  

    /* Stall条件 */     
    wire rs1_stall =  rs1use_ID && rs1_ID == rd_EXE && rd_EXE != 0 && 
                      hazard_optype_EXE == hazard_at_LOAD;

    wire rs2_stall =  rs2use_ID && rs2_ID == rd_EXE && rd_EXE != 0 && 
                      hazard_optype_EXE == hazard_at_LOAD &&
                      hazard_optype_ID != hazard_at_STORE;
    
    wire is_stall = rs1_stall | rs2_stall;

    assign PC_EN_IF = ~is_stall;
    assign reg_FD_stall = is_stall;
    assign reg_FD_flush = Branch_ID;
    assign reg_DE_flush = is_stall;
    assign reg_FD_EN = 1'b1;
    assign reg_DE_EN = 1'b1;
    assign reg_EM_EN = 1'b1;
    assign reg_EM_flush = 1'b0;
    assign reg_MW_EN = 1'b1;

    assign forward_ctrl_A = {2{rs1_forward_EX}}     & hazard_at_ALU   |
                            {2{rs1_forward_EX_MEM}} & hazard_at_LOAD  |
                            {2{rs1_forward_MEM_WB}} & hazard_at_STORE   ;

    assign forward_ctrl_B = {2{rs2_forward_EX}}     & hazard_at_ALU   |
                            {2{rs2_forward_EX_MEM}} & hazard_at_LOAD  |
                            {2{rs2_forward_MEM_WB}} & hazard_at_STORE   ;

    assign forward_ctrl_ls = rs2_EXE == rd_MEM && hazard_optype_EXE == hazard_at_STORE &&
                             hazard_optype_MEM == hazard_at_LOAD;

endmodule 