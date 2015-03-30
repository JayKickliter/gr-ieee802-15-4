module ieee802_15_4

using Match


export  PacketSink,
        Modulation,
        BPSK,
        OQPSK,
        exec


abstract SyncState
type SyncSearch <: SyncState end
type HeaderSearch   <: SyncState end
type CollectPayload <: SyncState end


abstract Modulation
type BPSK <: Modulation end
type OQPSK <: Modulation end

const STATE_SYNC_SEARCH = 0
const STATE_HAVE_SYNC   = 1
const STATE_HAVE_HEADER = 2
const MAX_PKT_LEN       = 127
const CHIP_MAP_BPSK     = Uint16[ 0b000100110101111, 0b111011001010000 ]
const CHIP_MASK_BPSK    = 0b0011111111111110



type PacketSink{M<:Modulation,S<:SyncState}
    modulation::Type{M}
    state::Type{S}
    chips_per_symbol  # how many chips per demodulated symbol. 15 For BPSK, 32 for OQPSK
    chip_map          # Chip to bit(s) mapping for modulation T
    sync_vector       # 802.15.4 standard is 4x 0 bytes and 1x0xA7
    threshold         # how many bits may be wrong in sync vector
    shift_reg         # used to look for sync_vector
    preamble_cnt      # count on where we are in preamble
    chip_cnt          # counts the chips collected
    header            # header bits
    headerbitlen_cnt  # how many so far
    packet            # assembled payload
    packet_byte       # byte being assembled
    packet_byte_index # which bit of d_packet_byte we're working on
    packetlen         # length of packet
    packetlen_cnt     # how many so far
    payload_cnt       # how many bytes in payload
    lqi               # Link Quality Information
    lqi_sample_count
end

function PacketSink( ::Type{BPSK}, threshold::Integer )
    modulation        = BPSK
    state             = SyncSearch
    chips_per_symbol  = 15
    chip_map          = CHIP_MAP_BPSK
    sync_vector       = 0xA7
    threshold         = threshold
    shift_reg         = zero(Uint16)
    preamble_cnt      = 0
    chip_cnt          = 0
    header            = zero(Uint16)
    headerbitlen_cnt  = 0
    packet            = zeros( Uint8, MAX_PKT_LEN )
    packet_byte       = zero( Uint8 )
    packet_byte_index = 0
    packetlen         = 0
    packetlen_cnt     = 0
    payload_cnt       = 0
    lqi               = 0
    lqi_sample_count  = 0
    input             = []
    input_count       = 0

    PacketSink( modulation, state, chips_per_symbol, chip_map, sync_vector, threshold, shift_reg, preamble_cnt, chip_cnt, header, headerbitlen_cnt, packet, packet_byte, packet_byte_index, packetlen, packetlen_cnt, payload_cnt, lqi, lqi_sample_count )
end


function chip_erros( a, b, mask )
    count_ones( (a & mask) $ (b & mask) )
end

function decode_chips( sink::PacketSink, chips::Integer )
    println( "decode_chips" )
    best_match = 0xFF
    min_errors = 16
    
    for symbol in 0:1
        reference_chips = CHIP_MAP_BPSK[ symbol+1 ]
        error_count     = chip_erros( chips, reference_chips, CHIP_MASK_BPSK )
        
        if error_count < min_errors
            best_match = symbol
            min_errors = error_count
        end
    end
    
    best_match::Uint8 = best_match >= sink.threshold ? 0xFF : best_match
end


function set_state( sink::PacketSink, ::Type{SyncSearch} )
    println( sink.state, " -> SyncSearch" )
    sink.state        = STATE_SYNC_SEARCH
    sink.shift_reg    = 0
    sink.preamble_cnt = 0
    sink.chip_cnt     = 0
    sink.packet_byte  = 0
end
    
function set_state( sink::PacketSink, ::Type{HeaderSearch} )
    println( sink.state, " -> HeaderSearch" )
    sink.state             = STATE_HAVE_SYNC;
    sink.packetlen_cnt     = 0
    sink.packet_byte       = 0
    sink.packet_byte_index = 0
    sink.lqi               = 0
    sink.lqi_sample_count  = 0
end

function set_state( sink::PacketSink, ::Type{CollectPayload} )
    println( sink.state, " -> CollectPayload" )
    sink.state             = CollectPayload
    sink.packetlen         = payload_len
    sink.payload_cnt       = 0
    sink.packet_byte       = 0
    sink.packet_byte_index = 0
end

function syncsearch( sink::PacketSink, input, input_idx )
    println( "syncsearch" )
    
    while input_idx <= length( input )
        sink.shift_reg = (sink.shift_reg << 1) | (input[input_idx] & 1)
        input_idx     += 10
        // Pickup where
        
    end
end


function headersearch( sink::PacketSink, input, input_idx )
    println( "headersearch" )

end

function collectpayload( sink::PacketSink, input, input_idx )
    println( "CollectPayload" )    
end




function exec( sink::PacketSink, input::Vector, input, input_idx )
    println( "exec" )
    
    input_idx = 1
    
    while input_idx <= length(input)

        @match sink.state begin
            SyncSearch      => input_idx = syncsearch( sink, input, input_idx )
            HeaderSearch    => input_idx = headersearch( sink, input, input_idx )
            CollectPayload  => input_idx = collectpayload( sink, input, input_idx )
        end

    end
end













end # module PacketSink


using ieee802_15_4

sink = PacketSink( BPSK, 10 )
