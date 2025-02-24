
module cache_2wsa_modified#(parameter AWIDTH        = 9,    //Address Bus width

parameter DWIDTH         = 32,

parameter CACHELINES     = 16,

parameter BLOCKSIZE         = 4,

parameter NUMOFSETS         = 8,

parameter VALIDBIT         = 1,

parameter DIRTYBIT         = 1,

parameter USEDBIT         = 1,

parameter MSIBIT         = 2,

parameter TAGWIDTH         = 6,

parameter BUS_INVALIDATE =2'b00,

parameter BUS_WRITE_MISS =2'b01,

parameter BUS_READ_MISS  =2'b10

)(

    input            clock,        //Clock input same as CPU and Memory controller(if MemController work on same freq.)

    input            reset_n,    //Active Low Asynchronous Reset Signal Input
    input   [1:0]           bus_requests, //from other caches: BUS_INVALIDATE=2'b00, BUS_WRITE_MISS=2'b01, BUS_READ_MISS=2'b10;

    input   [AWIDTH-1:0]    bus_request_mem_address, //the position they reffer

    input    [DWIDTH-1:0]    data_in,    //Parameterized Bi-directional Data bus from CPU

    input    [DWIDTH-1:0]    data_mem_in,    //Parameterized Bi-directional Data bus to Main Memory

    input    [AWIDTH-1:0]    addr_cpu,    //Parameterized Address bus from CPU

    output    reg[AWIDTH-1:0]    addr_mem,    //Parameterized Address bus to Main Memory
    input            rd_cpu,        //Active High Read signal from CPU

    input            wr_cpu,        //Active High WRITE signal from CPU
    output    reg        rd_mem,        //Active High Read signal to Main Memory

    output    reg        wr_mem,        //Active High Write signal to Main Memory
    output    reg        stall_cpu,    //Active High Stall Signal to CPU, to halt the CPU while undergoing any other operation

    input            ready_mem,    //Active High Ready signal from Main memory, to know the status of memory

    output reg [DWIDTH-1:0]    data_out,

    output reg [DWIDTH-1:0] data_mem_out

);

typedef enum{​​​​​​IDLE = 0, READ, WRITE, READMM, WAITFORMM, UPDATEMM, UPDATECACHE }​​​​​​ name_of_state;

// Parameters
 
// State Machine Parameters
/*localparam    IDLE        = 3'd0,    //Please read Description for explanation of States and their operation

        READ        = 3'd1,

        WRITE        = 3'd2,

        READMM        = 3'd3,

        WAITFORMM    = 3'd4,

        UPDATEMM    = 3'd5,

        UPDATECACHE    = 3'd6;*/
// Internal Wires and Registers
wire    [10:0]    tagdata;

wire    [2:0]    index;

wire    [1:0]   MSI_bit0;

wire    [1:0]   MSI_bit1;

reg    [DWIDTH-1:0] rdata_byte;

reg    [DWIDTH-1:0] wdata_byte;

reg    [DWIDTH-1:0] wmem_byte;

reg    [(DWIDTH)-1:0] rmem_4byte;

reg    [(DWIDTH)-1:0] wmem_4byte;    

logic [1:0] state_in;

logic cpu_write_hit;

logic cpu_read_hit;

logic cpu_write_miss;

logic cpu_read_miss;

logic write_hit;

logic read_hit;

logic write_miss;

logic read_miss;

logic miss;

//reg    [3:0] count;    //To count byte transfer between Cache and memory during read and write memory operation, used as shift register.
reg    rdwr; // If read then '1', if write the '0'

reg    we0;    //Active High Write Enable for DATA RAM 0

reg    we1;    //Active High Write Enable for DATA RAM 1

reg    wet0;    //Active High Write Enable for TAG RAM 0

reg    wet1;    //Active High Write Enable for TAG RAM 1
reg    update_flag; // Internal flag, SET when enters Update MM state. It is used to make reuse of WAITFORMM state for both READMM and UPDATEMM             //states
// Internal Signals derived from respective data or address buses

wire    hit;

wire    hit_w0;

wire    hit_w1;
wire    valid;

wire    vw0;

wire    vw1;
wire    uw0;

wire    uw1;
wire    dirty;

wire    dw0;

wire    dw1;
wire    [(MSIBIT+VALIDBIT+USEDBIT+DIRTYBIT+TAGWIDTH-1):0]    rtag0; //14-bits

wire    [(MSIBIT+VALIDBIT+USEDBIT+DIRTYBIT+TAGWIDTH-1):0]    rtag1;

wire    [(DWIDTH)-1:0] rdata0;    

wire    [(DWIDTH)-1:0] rdata1;
//wire    [DWIDTH-1:0]         bytew0;

//wire    [DWIDTH-1:0]         bytew1;
reg    [(DWIDTH)-1:0] rdata;

reg    [(DWIDTH)-1:0] wdata;

reg    [(DWIDTH)-1:0] strdata0;

reg    [(DWIDTH)-1:0] strdata1;

reg    [(MSIBIT+VALIDBIT+USEDBIT+DIRTYBIT+TAGWIDTH-1):0] wtag0;

reg    [(MSIBIT+VALIDBIT+USEDBIT+DIRTYBIT+TAGWIDTH-1):0] wtag1;

reg    [(MSIBIT+VALIDBIT+USEDBIT+DIRTYBIT+TAGWIDTH-1):0] strtag0;

reg    [(MSIBIT+VALIDBIT+USEDBIT+DIRTYBIT+TAGWIDTH-1):0] strtag1;
reg    [AWIDTH-1:0]         addrlatch;
// State Variables

reg [2:0] state;

name_of_state state_name;
// bus assignment
wire [5:0] cache_tag_attending_bus = bus_request_mem_address[8:3];//decomposing the bus address input into index and tag

wire [2:0] cache_index_attending_bus = bus_request_mem_address[2:0];

wire[]1:0] bus_rtag0 = 

//wire [1:0] coherency_state_attending_bus = cache[cache_index_attending_bus][21:20];

// Combinational Logic
assign tagdata = (state == IDLE) ? addr_cpu[8:3] : addrlatch[8:3];

assign index   = (state == IDLE) ? addr_cpu[2:0] : addrlatch[2:0];

//assign bytsel  = (state == IDLE) ? addr_cpu[1:0] : addrlatch[1:0];
assign vw0 = rtag0[8];

assign vw1 = rtag1[8];

assign valid = vw0 & vw1;
assign uw0  =rtag0[7];

assign uw1  =rtag1[7];
assign dw0 = rtag0[6];

assign dw1 = rtag1[6];

assign dirty = dw0 | dw1;
assign hit_w0 = vw0 & (tagdata == rtag0[5:0]) & (|rtag0[10:9]);

assign hit_w1 = vw1 & (tagdata == rtag1[5:0]) & (|rtag1[10:9]);

assign hit = hit_w0 | hit_w1;
assign MSI_bit0= rtag0[10:9];

assign MSI_bit1= rtag1[10:9];
assign data_mem_out = wmem_byte;

assign stall_cpu = ~hit;

assign miss = ~hit;
assign write_hit = hit & wr_cpu;

assign read_hit = hit & rd_cpu;

assign write_miss = miss & wr_cpu;

assign read_miss = miss & rd_cpu;
always@(write_hit or read_hit or write_miss or read_miss) begin

case ({​​​​​​write_hit,read_hit,write_miss,read_miss}​​​​​​)

4'b0001:begin cpu_read_miss  = 1'b1; cpu_write_miss = 1'b0; cpu_read_hit =1'b0; cpu_write_hit = 1'b0; end

4'b0010:begin cpu_read_miss  = 1'b0; cpu_write_miss = 1'b1; cpu_read_hit =1'b0; cpu_write_hit = 1'b0; end

4'b0100:begin cpu_read_miss  = 1'b0; cpu_write_miss = 1'b0; cpu_read_hit =1'b1; cpu_write_hit = 1'b0; end

4'b1000:begin cpu_read_miss  = 1'b0; cpu_write_miss = 1'b0; cpu_read_hit =1'b0; cpu_write_hit = 1'b1; end

endcase

end

//assign cpu_write_hit= write_hit? 1:0;

//assign cpu_read_hit= read_hit ? 1:0;

//assign cpu_write_miss= write_hit? 0:1;

//assign cpu_read_miss= read_hit? 0:1;

// Cache Controller State Machine and Logic
always@(posedge clock or negedge reset_n)

begin

    if(!reset_n)

    begin

        addrlatch      <= 'd0;

        addr_mem       <= 'd0;

        rd_mem           <= 'd0;

        wr_mem           <= 'd0;

        stall_cpu      <= 'd0;

        state           <= IDLE;

        state_name     <= IDLE;

        rdata_byte     <= 'd0;

        wdata_byte     <= 'd0;

        wmem_byte      <= 'd0;

        rmem_4byte     <= 'd0;

        wmem_4byte     <= 'd0;

        wdata           <= 'd0;

        wtag0           <= 'd0;

        wtag1           <= 'd0;

        we0               <= 1'd0;

        we1               <= 1'd0;

        wet0           <= 1'd0;

        wet1           <= 1'd0;

        rdwr           <= 1'd1;

        strdata0       <= 'd0;

        strdata1       <= 'd0;

        strtag0        <= 'd0;

        strtag1           <= 'd0;

        rdata           <= 'd0;

        //count           <= 4'd0;

        update_flag    <= 1'd0;

        cpu_write_hit  <= 'b0;

        cpu_read_hit   <= 'b0;

        cpu_write_miss  <= 'b0;

        cpu_read_miss   <= 'b0;

        state_in        <= 'b0;

    end

    else

    begin

        case(state)
            IDLE    :    begin

                    addrlatch       <= addr_cpu;

                    we0               <= 1'd0;

                    we1               <= 1'd0;

                    wet0           <= 1'd0;

                    wet1           <= 1'd0;

                    //stall_cpu      <= 1'd0;

                    rd_mem           <= 1'd0;

                    wr_mem           <= 1'd0;

//                    rdata_byte<= 8'd0;

                    wmem_byte      <= 'd0;

                    rmem_4byte     <= 'd0;

                    wdata           <= 'd0;

                    wtag0           <= 'd0;

                    wtag1           <= 'd0;

                    update_flag    <= 1'd0;

                    //cpu_write_hit  <= 'b0;

                    //cpu_read_hit   <= 'b0;

                    //cpu_write_miss  <= 'b0;

                    //cpu_read_miss   <= 'b0;

                    //count           <= 4'd0;
                    if(rd_cpu)

                    begin

                        state        <= READ;

                        state_name  <= READ;

                        data_out    <= rdata_byte;

                        rdwr        <= 1'd1;

                    end

                    else if(wr_cpu)

                    begin

                        state        <= WRITE;

                        state_name  <= WRITE;

                        wdata_byte    <= data_in;

                        rdwr        <= 1'd0;

                    end

                    else

                        begin

                        state         <= state;

                        state_name   <= IDLE;

                        end

                    end
            READ    :    begin

                    we0 <= 1'd0;

                    we1 <= 1'd0;

                    case(hit)

                        1'd0:    begin

                            strtag0       <= rtag0;

                            strtag1       <= rtag1;

                            strdata0   <= rdata0;

                            strdata1   <= rdata1;

                            wet0       <= 1'd0;

                            wet1       <= 1'd0;

                            //cpu_read_miss =1'b1;

                            if(ready_mem)

                                if(valid & dirty)

                                    begin

                                    state      <= UPDATEMM;

                                    state_name <= UPDATEMM;

                                    end

                                else

                                    begin

                                    state      <= READMM;

                                    state_name <= READMM;

                                    end

                            else

                                begin

                                state          <= state;

                                state_name     <= READ;

                                end

                            end
                        1'd1:    begin

                            state      <= IDLE;

                            state_name <= IDLE;

                            wet0       <= 1'd1;

                            wet1       <= 1'd1;

                            //cpu_read_hit = 1'b1;

                                if(hit_w0)

                                begin

                                    rdata_byte  <= rdata0;

                                    state_in <= MSI_bit0;

                                    if(uw0)

                                        wtag0   <= {​​​​​​state_next,rtag0[8:0]}​​​​​​;

                                    else

                                        wtag0   <= {​​​​​​state_next,rtag0[8],1'd1,rtag0[6:0]}​​​​​​;

                                    if(uw1)

                                        wtag1   <= {​​​​​​state_next,rtag1[8:0]}​​​​​​;

                                    else

                                        wtag1   <= {​​​​​​state_next,rtag1[8],1'd1,rtag1[6:0]}​​​​​​;

                                end

                                else

                                begin

                                    rdata_byte  <= rdata1;

                                    state_in <= MSI_bit1;

                                    if(uw1)

                                        wtag1   <= {​​​​​​state_next,rtag1[8],1'd0,rtag1[6:0]}​​​​​​;

                                    else

                                        wtag1   <= {​​​​​​state_next,rtag1[8:0]}​​​​​​;

                                    if(uw0)

                                        wtag0   <= {​​​​​​state_next,rtag0[8],1'd0,rtag0[6:0]}​​​​​​;

                                    else

                                        wtag0   <= {​​​​​​state_next,rtag0[8:0]}​​​​​​;

                                end

                            end

                    endcase

                    end
            WRITE    :    begin

                    case(hit)

                        1'd0:    begin

                            strtag0       <= rtag0;

                            strtag1       <= rtag1;

                            strdata0   <= rdata0;

                            strdata1   <= rdata1;

                            //cpu_write_miss = 1'b1;
                            if(ready_mem)

                                if(valid & dirty)

                                    begin

                                    state        <= UPDATEMM;

                                    state_name   <= UPDATEMM;

                                    end

                                else

                                    begin

                                    state       <= READMM;

                                    state_name  <= READMM;

                                    end

                            else

                                begin

                                state       <= state;

                                state_name  <= WRITE;

                                end
                            end
                        1'd1:    begin

                            state       <= IDLE;

                            state_name  <= IDLE;

                            wet0         <= 1'd1;

                            wet1         <= 1'd1;

                            //cpu_write_hit =1'd1;

                                if(hit_w0)

                                    begin

                                    we0        <= 1'd1;

                                    wdata   <= wdata_byte;

                                    state_in <= MSI_bit0;

                                    if(uw0)

                                        wtag0 <= {​​​​​​state_next,rtag0[8:7],1'd1,rtag0[5:0]}​​​​​​;

                                    else

                                        wtag0 <= {​​​​​​state_next,rtag0[8],1'd1,1'd1,rtag0[5:0]}​​​​​​;

                                    if(uw1)

                                        wtag1 <= {​​​​​​state_next,rtag1[8:0]}​​​​​​;

                                    else

                                        wtag1 <= {​​​​​​state_next,rtag1[8],1'd1,rtag1[6:0]}​​​​​​;

                                    end

                                else

                                    begin

                                    we1        <= 1'd1;

                                    wdata   <= wdata_byte;

                                    state_in <= MSI_bit1;

                                    if(uw1)

                                        wtag1 <= {​​​​​​state_next,rtag1[8],1'd0,1'd1,rtag1[5:0]}​​​​​​;

                                    else

                                        wtag1 <= {​​​​​​state_next,rtag1[8:7],1'd1,rtag1[5:0]}​​​​​​;

                                    if(uw0)

                                        wtag0 <= {​​​​​​state_next,rtag0[8],1'd0,rtag0[6:0]}​​​​​​;

                                    else

                                        wtag0 <= {​​​​​​state_next,rtag0[8:0]}​​​​​​;

                                    end

                            end

                    endcase

                    end

        READMM    :    begin

                    addr_mem            <= addrlatch;

                    update_flag         <= 1'd0;

                        if(ready_mem)

                        begin

                            rd_mem      <= 1'd1;

                            state       <= WAITFORMM;

                            state_name  <= WAITFORMM;

                        end

                        else

                        begin

                            rd_mem       <= 1'd0;

                            state        <= state;

                            state_name   <= READMM;

                        end

                    end
            WAITFORMM :    begin

                        rd_mem <= 1'd0;

                        wr_mem <= 1'd0;

                        if(ready_mem)

                        begin                        

                            if(update_flag)

                            begin

                            state      <= READMM;

                            state_name <= READMM;

                            end

                            else

                            begin

                            state      <= UPDATECACHE;

                            state_name <= UPDATECACHE;

                            end

                            rd_mem <= 1'd0;

                            wr_mem <= 1'd0;

                        end

                        else

                        begin

                            state <= state;

                            state_name <= WAITFORMM;

                        end

                    end
UPDATEMM :    begin

                        update_flag    <= 1'd1;

                        if(uw0)

                        begin

                            addr_mem   <= {​​​​​​strtag1[5:0],addrlatch[2:0]}​​​​​​;

                            wmem_byte <= strdata1;

                        end

                        else

                        begin

                            addr_mem   <= {​​​​​​strtag0[5:0],addrlatch[2:0]}​​​​​​;

                            wmem_byte <= strdata0;

                        end

                        if(ready_mem)

                        begin

                            wr_mem     <= 1'd1;

                            state      <= WAITFORMM;

                            state_name <= WAITFORMM;

                        end

                        else

                        begin

                            wr_mem     <= 1'd0;

                            state      <= state;

                            state_name <= UPDATEMM;

                        end

                    end
            UPDATECACHE:    begin

                            update_flag  <= 1'd0;

                            wdata        <= data_mem_in;

                            state        <= IDLE;

                            state_name   <= IDLE;

                            if(uw0)

                            begin

                                wtag1 <= {​​​​​​state_next,1'd1,1'd0,1'd0,addrlatch[8:3]}​​​​​​;

                                wtag0 <= {​​​​​​state_next,strtag0[8],1'd0,strtag0[6:0]}​​​​​​;

                                we1   <= 1'd1;

                                we0   <= 1'd0;

                                wet0  <= 1'd1;

                                wet1  <= 1'd1;

                                state_in <= 'b0;

                            end

                            else

                            begin

                                wtag0 <= {​​​​​​state_next,1'd1,1'd1,1'd0,addrlatch[8:3]}​​​​​​;

                                wtag1 <= {​​​​​​state_next,strtag1[8],1'd1,strtag1[6:0]}​​​​​​;

                                we0   <= 1'd1;

                                we1   <= 1'd0;

                                wet0  <= 1'd1;

                                wet1  <= 1'd1;

                                state_in <= 'b0;

                            end

                        end


                        default:    begin

                            addrlatch <= 'd0;

                            addr_mem  <= 'd0;

                            rd_mem      <= 'd0;

                            wr_mem      <= 'd0;

                            stall_cpu <= 'd0;

                            state      <= IDLE;

                            rdata_byte<= 'd0;

                            wdata_byte<= 'd0;

                            wmem_byte <= 'd0;

                            rmem_4byte<= 'd0;

                            wmem_4byte<= 'd0;

                            wdata      <= 'd0;

                            wtag0      <= 'd0;

                            wtag1      <= 'd0;

                            we0          <= 1'd0;

                            we1          <= 1'd0;

                            wet0      <= 1'd0;

                            wet1      <= 1'd0;

                            rdwr      <= 1'd1;

                            strdata0  <= 'd0;

                            strdata1  <= 'd0;

                            strtag0   <= 'd0;

                            strtag1      <= 'd0;

                            rdata      <= 'd0;

                            //cpu_write_hit  <= 'b0;

                            //cpu_read_hit   <= 'b0;

                            //cpu_write_miss  <= 'b0;

                            //cpu_read_miss   <= 'b0;
                    end

        endcase

    end

end
// Instantiation of Tag RAM for Way 0
defparam tr0.AWIDTH = 3;

defparam tr0.DWIDTH = MSIBIT+VALIDBIT+USEDBIT+DIRTYBIT+TAGWIDTH;
ram_sync_read_t0 tr0 (

            .clock(clock),

            .addr(index),

            .din(wtag0),

            .we(wet0),

            .dout(rtag0)

            );
// Instantiation of Tag RAM for Way 0
defparam tr1.AWIDTH = 3;

defparam tr1.DWIDTH = MSIBIT+VALIDBIT+USEDBIT+DIRTYBIT+TAGWIDTH;
ram_sync_read_t1 tr1 (

            .clock(clock),

            .addr(index),

            .din(wtag1),

            .we(wet1),

            .dout(rtag1)

            );
// Instantiation Data RAM for Way 0
defparam dr0.AWIDTH = 3;

defparam dr0.DWIDTH = DWIDTH;
ram_sync_read_d0 dr0 (

            .clock(clock),

            .addr(index),

            .din(wdata),

            .we(we0),

            .dout(rdata0)

            );
// Instantiation Data RAM for Way 1
defparam dr1.AWIDTH = 3;

defparam dr1.DWIDTH = DWIDTH;
ram_sync_read_d1 dr1 (

            .clock(clock),

            .addr(index),

            .din(wdata),

            .we(we1),

            .dout(rdata1)

            );

// Instantiation of FSM for MSI_CPU            

FSM_MSI_CPU_requests_controller MSI_CPU_controller(

    .state_in(state_in),

    .cpu_write_hit(cpu_write_hit),

    .cpu_read_hit(cpu_read_hit),

    .cpu_write_miss(cpu_write_miss),

    .cpu_read_miss(cpu_read_miss),

    .write_back_block_next(),

    .state_next(state_next),

    .bus_next(bus_next)

    );    

// Instantiation of FSM for MSI_BUS    
FSM_MSI_BUS_controller      FSM_MSI_BUS(
   .state_in(coherency_state_attending_bus),

   .bus_write_miss(bus_requests==BUS_WRITE_MISS?1'b1:1'b0),

   .bus_read_miss(bus_requests==BUS_READ_MISS?1'b1:1'b0),

   .bus_invalidate(bus_requests==BUS_INVALIDATE?1'b1:1'b0),

   .abort_mem_access_next(bus_controler_abort_mem_access),

   .write_back_block_next(bus_controler_write_back),

   .state_next(state_next_bus)

    );


// END OF MODULE

endmodule
