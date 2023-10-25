`timescale 1ns / 1ps

module ExceptionUnit(
    input clk, rst,
    input csr_rw_in,
    input[1:0] csr_wsc_mode_in,
    input csr_w_imm_mux,
    input[11:0] csr_rw_addr_in,
    input[31:0] csr_w_data_reg,
    input[4:0] csr_w_data_imm,
    output[31:0] csr_r_data_out,

    input interrupt,
    input illegal_inst,
    input l_access_fault,
    input s_access_fault,
    input ecall_m,

    input mret,

    input[31:0] epc_cur,
    input[31:0] epc_next,
    output[31:0] PC_redirect,
    output redirect_mux,

    output reg_FD_flush, reg_DE_flush, reg_EM_flush, reg_MW_flush, 
    output RegWrite_cancel
);

    reg[11:0] csr_raddr, csr_waddr;
    reg[31:0] csr_wdata;
    reg csr_w;
    reg[1:0] csr_wsc;
    initial begin
        csr_w = 1'b0;
        csr_raddr = 12'b0;
    end

    wire[31:0] mstatus;     // mstatus总会在时钟正边沿的时候更新???

    CSRRegs csr(.clk(clk),.rst(rst),.csr_w(csr_w),.raddr(csr_raddr),.waddr(csr_waddr),
        .wdata(csr_wdata),.rdata(csr_r_data_out),.mstatus(mstatus),.csr_wsc_mode(csr_wsc));

    //According to the diagram, design the Exception Unit

    /* 状???机状??? */
    reg[1:0] next_state;    // 状???机当前状?
    localparam STATE_IDLE   = 2'b00;
    localparam STATE_MEPC   = 2'b01;
    localparam STATE_MCAUSE = 2'b10;
    initial next_state = STATE_IDLE;

    /* CSR寄存器地?? */
    localparam mstatus_addr = 12'h300; // {5'h6 , 7'b0000000}
    localparam mtvec_addr   = 12'h305; // {5'h6 , 7'b0000101}
    localparam mepc_addr    = 12'h341; // {5'h6 , 7'b1000001}
    localparam mcause_addr  = 12'h342; // {5'h6 , 7'b1000010}
    localparam mtval_addr   = 12'h343; // {5'h6 , 7'b1000011}
    localparam mip_addr     = 12'h344; // {5'h6 , 7'b1000100}

    /* CSR使能信号 */
    wire mie;
    assign mie = mstatus[3];

    /* 异常类型分类 */
    // wire MRET = mret; (已有)
    wire interrupt_true = interrupt & mie;
    wire [3:0] exception_set = {ecall_m, s_access_fault, l_access_fault, illegal_inst} & {4{mie}};
    wire abnormal_true = mret | interrupt_true | (|exception_set);

    /* FLUSH使能 */
    reg flush_MW, flush_def;
    assign reg_FD_flush = flush_def;
    assign reg_DE_flush = flush_def;
    assign reg_EM_flush = flush_def;
    assign reg_MW_flush = flush_MW;

    /* 中间?? */
    reg[1:0] curr_mode;
    reg[31:0] epc;
    reg[31:0] cause;
    initial curr_mode <= 2'b00;
    

    /* 输出?? */
    localparam next_PC = 1'b0;
    localparam redirect = 1'b1;

    reg ret_mux;
    // reg [31:0] ret_PC;
    reg reg_w_cancel;
    initial begin
        ret_mux = next_PC;
        reg_w_cancel = 1'b0;
    end
    assign redirect_mux = ret_mux;
    assign PC_redirect = csr_r_data_out;
    assign RegWrite_cancel = reg_w_cancel;

    /* 中断使用的状态机 */
    always@(negedge clk) begin
        case(next_state)
            STATE_IDLE: begin
                /* 1. STATE_IDLE ??(exception or interruption) STATE_MEPC */
                if (interrupt_true | (|exception_set)) begin
                    /* 1.1. write mstatus: */
                    /* 1.1.1. 调整写使?? */
                    csr_w = 1'b1;

                    /* 1.1.2. 调整写地?? -> CSR寄存?? */
                    csr_waddr = mstatus_addr;

                    /* 1.1.3. 调整写内?? -> MPIE = MIE, MIE = 0, 
                     *        MPP = curr_mode, curr_mode = 2'b11*/
                    csr_wdata = mstatus;
                    csr_wdata[7] = csr_wdata[3];
                    csr_wdata[3] = 1'b0;
                    csr_wdata[12:11] = curr_mode;
                    curr_mode = 2'b11;

                    /* 1.1.4. 调整写方?? */
                    csr_wsc = 2'b00;

                    /* 1.2. flush all the pipeline registers */
                    flush_MW = 1'b1;
                    flush_def = 1'b1;

                    /* 1.3. record epc and cause */
                    /* exception */
                    if (|exception_set) begin
                        /* if exception (not interrupt), cancal regwrite */
                        reg_w_cancel = 1'b1;
                        epc = epc_cur;
                        case(exception_set)
                            /* illegal_inst */
                            4'b0001:begin
                                cause = 32'd2;
                            end
                            /* l_access_fault */
                            4'b0010:begin
                                cause = 32'd5;
                            end
                            /* s_access_fault */
                            4'b0100:begin
                                cause = 32'd7;
                            end
                            /* ecall */
                            4'b1000:begin
                                cause = 32'd11;
                            end
                        endcase
                    end
                    /* interrupt */
                    else begin
                        reg_w_cancel = 1'b0;
                        epc = epc_next;
                        cause = {1'b1, 31'd11};
                    end

                    /* other */
                    next_state = STATE_MEPC;
                end

                /* 2. STATE_IDLE ?? (mret) STATE_IDLE */
                else if (mret) begin
                    /* 2.1. write mstatus */
                    /* 2.1.1. 调整写使?? */
                    csr_w = 1'b1;

                    /* 2.1.2. 调整写地?? -> CSR寄存?? */
                    csr_waddr = mstatus_addr;

                    /* 2.1.3. 调整写内?? -> MPIE = MIE, MIE = 0, 
                     *        curr_mode = MPP, MPP = */
                    csr_wdata = mstatus;
                    csr_wdata[3] = csr_wdata[7];
                    csr_wdata[7] = 1'b1;
                    curr_mode = csr_wdata[12:11];
                    csr_wdata[12:11] = 2'b11;

                    /* 2.1.4. 调整写方?? */
                    csr_wsc = 2'b00;

                    /* 2.2. read mepc */
                    csr_raddr = mepc_addr;
                    // ret_PC = mepc_addr;

                    /* 2.3. set redirect pc mux (next cycle pc ?? mepc) */
                    ret_mux = redirect;

                    /* 2.4. flush pipeline registers (EM, DE, FD) */
                    flush_def = 1'b1;
                    flush_MW = 1'b0;

                    next_state = STATE_IDLE;
                end
                
                /* 3. STATE_IDLE ?? (csr insts) STATE_IDLE */
                else if(csr_rw_in) begin
                    csr_w     = 1 ;
                    csr_raddr = csr_rw_addr_in;
                    csr_waddr = csr_rw_addr_in;
                    csr_wdata = csr_w_imm_mux == 1 ? 
                                 {27'b0, csr_w_data_imm} : csr_w_data_reg;
                    csr_wsc   = csr_wsc_mode_in;

                    /* other */
                    reg_w_cancel = 1'b0;
                    flush_def = 1'b0;
                    flush_MW = 1'b0;
                    ret_mux = next_PC;
                end

                /* 4. STATE_IDLE ?? (other) STATE_IDLE */
                else begin
                    csr_w = 0;
                    next_state = STATE_IDLE;

                    /* other */
                    reg_w_cancel = 1'b0;
                    flush_def = 1'b0;
                    flush_MW = 1'b0;
                    ret_mux = next_PC;
                end
            end

            /* 5. STATE_MEPC ?? STATE_MCAUSE */
            STATE_MEPC: begin
                /* 5.1. write epc to mepc */
                csr_w = 1'b1;
                csr_waddr = mepc_addr;
                csr_wdata = epc;
                csr_wsc = 2'b00;

                /* 5.2. read mtvec */
                csr_raddr = mtvec_addr;

                /* 5.3. set redirect pc mux (next cycle pc ?? mtvec) */
                ret_mux = redirect;
                // ret_PC = csr_r_data_out & 32'hFFFFFFFC;    // [1:0]??0

                next_state <= STATE_MCAUSE;

                /* other */
                reg_w_cancel = 1'b0;
                flush_def = 1'b0;
                flush_MW = 1'b0;
            end

            /* 6. STATE_MCAUSE ?? STATE_IDLE */
            STATE_MCAUSE: begin
                /* write cause to mcause */
                csr_w       = 1'b1;
                csr_waddr   = mcause_addr;
                csr_wdata   = cause;
                csr_wsc     = 2'b00;
                next_state  = STATE_IDLE;

                ret_mux = next_PC;
                
                /* other */
                reg_w_cancel = 1'b0;
                flush_def    = 1'b0;
                flush_MW     = 1'b0;
            end
        endcase
    end

endmodule