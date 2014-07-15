// Parameters of the traffic

// Width of the id of a flow
`define FLOW_ID_WIDTH 104
// Width of the lengh of a packet
`define PKT_LENGTH_WIDTH 16
// Width of the class of a flow
`define FLOW_CLASS_WIDTH 8
// Width of a flow counter (go to `SVM_PACKETS_SIZE_OFFSET + `SVM_PARAM_COUNT + 2)
`define FLOW_PKT_COUNTER_WIDTH 3
// Width of a word in one RAM used for flows
`define FLOW_RAM_WORD_WIDTH 72
// Width of the address to read one flow data in one RAM
// and estimated latency (used as FIFO size for pipelines)
`ifdef NETFPGA_1G
    // NetFPGA 1G SRAM memory
    `define FLOW_RAM_ADDR_WIDTH 19
    `define FLOW_RAM_LATENCY_BITS 3
`else `ifdef COMBOV2_10G2
    // Combov2 memory (two memories)
    `define FLOW_RAM_ADDR_WIDTH 21
    `define FLOW_RAM_LATENCY_BITS 4
`else `ifdef SIMULATED_HW
    // Small simulated BlockRAM memory
    `define FLOW_RAM_ADDR_WIDTH 12
    `define FLOW_RAM_LATENCY_BITS 4
`else
    // Synthesizable BlockRAM memory
    `define FLOW_RAM_ADDR_WIDTH 13
    `define FLOW_RAM_LATENCY_BITS 1
`endif `endif `endif
// Width of the RAM address that should alternate at each step to use the full RAM bandwidth
`define FLOW_RAM_ADDR_ALT_WIDTH 1
// Width of the RAM address that can be changed randomly at each read/write
`define FLOW_RAM_ADDR_RAND_WIDTH (`FLOW_RAM_ADDR_WIDTH - `FLOW_RAM_ADDR_ALT_WIDTH)
// Number of hashes to compute for one storage
`define FLOW_RAM_HASHES 8
// Number of bits required to address a hash (ln2(FLOW_RAM_HASHES-1))
`define FLOW_RAM_HASH_ID_WIDTH 3
// Width of the computed hash
`define FLOW_RAM_FULL_HASH_WIDTH 32
// Width of the id used to modify a flow
`define FLOW_RAM_EDIT_ID_WIDTH (`FLOW_RAM_FULL_HASH_WIDTH*`FLOW_RAM_HASHES + `FLOW_RAM_HASHES)
// Width of the timestamp of a flow
`define FLOW_TIMESTAMP_WIDTH 5
// Duration of a timestamp unit in clock cycles (~ 1 min.)
`define FLOW_TIMESTAMP_UNIT 1000000000 * 8
// Width of a timestamp unit counter
`define FLOW_TIMESTAMP_UNIT_WIDTH 33
// Number of timestamp units to wait before timeout
`define FLOW_TIMESTAMP_TIMEOUT 10
// Width of the redundancy number of a flow (0 to FLOW_RAM_HASHES-1)
`define FLOW_REDUNDANCY_WIDTH 3
// Width of the flow data (without hash, timestamp)
`define FLOW_DATA_WIDTH (`FLOW_RAM_WORD_WIDTH - ((`FLOW_RAM_FULL_HASH_WIDTH - `FLOW_RAM_ADDR_RAND_WIDTH) + `FLOW_TIMESTAMP_WIDTH + `FLOW_REDUNDANCY_WIDTH))
