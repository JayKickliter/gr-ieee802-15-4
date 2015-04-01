module ieee802_15_4

using Match


export  PacketSink,
        PacketSource,
        Modulation,
        BPSK,
        OQPSK,
        spread,
        CHIP_MAP_BPSK,
        make_packet,
        exec


abstract PHYState
type SyncOnZero     <: PHYState end
type SFDSearch      <: PHYState end
type HeaderSearch   <: PHYState end
type PayloadCollect <: PHYState end


abstract Modulation
type BPSK  <: Modulation end
type OQPSK <: Modulation end

const MAX_PKT_LEN       = 127
const CHIP_MAP_BPSK     = Uint16[ 0b000100110101111, 0b111011001010000 ]
const CHIP_MASK_BPSK    = 0b0011111111111110



type PacketSink{M}
    modulation::Type{M}      # BPSK, OQPSK, etc
    state::Type              # what is our PHYState
    chips_per_symbol::Int    # how many chips per demodulated symbol. 15 For BPSK, 32 for OQPSK
    chip_map::Vector{Uint16} # Chip to bit(s) mapping for modulation T
    sync_sequence::Uint16    # 802.15.4 standard is 4x 0 bytes and 1x0xA7, we will ignore the first byte
    threshold::Int           # how many bits may be wrong in sync vector
    sync_shift_reg::Uint16   # decoded chips are shifted in, and compared against sync_sequence
    chip_shift_reg::Uint16   # chips are shifted in and decoded to look for first 0
    chip_shift_count::Int    # how many chips have we shifted into chip_shift_reg
    last_diff_enc_bit::Int   # previous not-yet-decoded bit, need to keep it to do differential decoding
    packet::Vector{Uint8}    # assembled payload
    packet_byte::Uint8       # byte being assembled
    packet_byte_count::Int   # which bit of d_packet_byte we're working on
    packetlen::Int           # length of packet
    packetlen_cnt::Int       # how many so far
    payload_cnt::Int         # how many bytes in payload
    input_idx::Int           # our location in the input vector
    packet_byte_bit_count::Int #   
end

function PacketSink( modType, threshold )
    modulation            = BPSK
    state                 = SyncOnZero
    chips_per_symbol      = 15
    chip_map              = CHIP_MAP_BPSK
    sync_sequence         = 0x00e5
    threshold             = threshold
    sync_shift_reg        = zero( Uint16 )
    chip_shift_reg        = zero( Uint16 )
    chip_shift_count      = 0
    last_diff_enc_bit     = 0
    packet                = zeros( Uint8, MAX_PKT_LEN )
    packet_byte           = zero( Uint8 )
    packet_byte_count     = 0
    packetlen             = 0
    packetlen_cnt         = 0
    payload_cnt           = 0
    input_idx             = 0
    packet_byte_bit_count = 0

    PacketSink( modulation,
                state,
                chips_per_symbol,
                chip_map,
                sync_sequence,
                threshold,
                sync_shift_reg,
                chip_shift_reg,
                chip_shift_count,
                last_diff_enc_bit,
                packet,
                packet_byte,
                packet_byte_count,
                packetlen,
                packetlen_cnt,
                payload_cnt,
                input_idx,
                packet_byte_bit_count )
end


function count_bit_diffs( a, b, mask )
    count_ones( (a & mask) $ (b & mask) )
end

function chips_to_bit( sink::PacketSink{BPSK}, chips::Integer )
    best_match = 0xFF
    min_errors = 16
    is_valid_seq = false

    for symbol in 0:1
        reference_chips = CHIP_MAP_BPSK[ symbol+1 ]
        error_count     = count_bit_diffs( chips, reference_chips, CHIP_MASK_BPSK )

        if error_count < min_errors
            best_match = symbol
            min_errors = error_count
        end
    end

    is_valid_seq = min_errors <= sink.threshold

    return ( is_valid_seq, best_match )
end


function set_state( sink::PacketSink{BPSK}, ::Type{SyncOnZero} )
    println( sink.state, " -> SyncOnZero" )
    sink.state             = SyncOnZero
    sink.last_diff_enc_bit = 0
    sink.sync_shift_reg    = zero( Uint16 )
    sink.chip_shift_reg    = zero( Uint16 )
    sink.chip_shift_count  = 0
    sink.packet_byte       = 0
end

function set_state( sink::PacketSink{BPSK}, ::Type{SFDSearch} )
    println( sink.state, " -> SFDSearch" )
    sink.state             = SFDSearch
    sink.last_diff_enc_bit = 0
    sink.chip_shift_reg    = zero( Uint16 )
    sink.chip_shift_count  = 0
    sink.packet_byte       = 0
end

function set_state( sink::PacketSink{BPSK}, ::Type{HeaderSearch} )
    println( sink.state, " -> HeaderSearch" )
    sink.state                 = HeaderSearch
    sink.packet_byte           = 0
    sink.packet_byte_bit_count = 0
    sink.packet_byte_count     = 0
end

function set_state( sink::PacketSink{BPSK}, ::Type{PayloadCollect} )
    println( sink.state, " -> PayloadCollect" )
    sink.state             = PayloadCollect
    # sink.packetlen         = payload_len
    # sink.payload_cnt       = 0
    # sink.packet_byte       = 0
    # sink.packet_byte_count = 0
end

function synconzero( sink::PacketSink{BPSK}, input::Vector )

    while sink.input_idx <= length( input )
        @printf( "SyncOnZero. input_idx: %d, chip_shift_reg: %s, chip_shift_count: %d, sync_shift_reg: %s\n", sink.input_idx, bin(sink.chip_shift_reg, 15),sink.chip_shift_count, hex(sink.sync_shift_reg,4))
        sink.chip_shift_reg    = uint16( (sink.chip_shift_reg >> 1) | ((input[sink.input_idx] & 1)<<14) )
        sink.input_idx        += 1
        sink.chip_shift_count += 1

        (is_valid_seq, diff_enc_bit) = chips_to_bit( sink, sink.chip_shift_reg )

        if is_valid_seq && diff_enc_bit == 0
            sink.sync_shift_reg    = (sink.sync_shift_reg << 1) | diff_enc_bit
            sink.chip_shift_count  = 0
            sink.chip_shift_reg    = zero( sink.chip_shift_reg )
            set_state( sink, SFDSearch )
            break
        end
    end

end

function sfdsearch( sink::PacketSink{BPSK}, input::Vector )

    while sink.input_idx <= length( input )
        @printf( "SFDSearch. input_idx: %d, chip_shift_reg: %s, chip_shift_count: %d, sync_shift_reg: %s\n", sink.input_idx, bin(sink.chip_shift_reg, 15),sink.chip_shift_count, hex(sink.sync_shift_reg,4))
        sink.chip_shift_reg    = uint16( (sink.chip_shift_reg >> 1) | ((input[sink.input_idx] & 1)<<14) )
        sink.input_idx        += 1
        sink.chip_shift_count += 1
        

        if sink.chip_shift_count == 15
            (is_valid_seq, diff_enc_bit) = chips_to_bit( sink, sink.chip_shift_reg )
            if !is_valid_seq
                set_state( sink, SyncOnZero )
                break
            end

            sink.sync_shift_reg   = uint16( (sink.sync_shift_reg << 1) | diff_enc_bit )

            sink.chip_shift_count = 0
            sink.chip_shift_reg   = zero( sink.chip_shift_reg )

            if sink.sync_shift_reg == sink.sync_sequence
                set_state( sink, HeaderSearch )
                break
            end

        end
    end

end



function headersearch( sink::PacketSink{BPSK}, input::Vector )
    while sink.input_idx <= length( input )
        @printf( "HeaderSearch. input_idx: %d, chip_shift_reg: %s, chip_shift_count: %d, packet_byte: %s\n", sink.input_idx, bin(sink.chip_shift_reg, 15),sink.chip_shift_count, hex(sink.packet_byte,4))    
        sink.chip_shift_reg    = uint16( (sink.chip_shift_reg >> 1) | ((input[sink.input_idx] & 1)<<14) )
        sink.input_idx        += 1
        sink.chip_shift_count += 1

        if sink.chip_shift_count == 15
            (is_valid_seq, diff_enc_bit) = chips_to_bit( sink, sink.chip_shift_reg )
            if !is_valid_seq
                set_state( sink, SyncOnZero )
                break
            end

            sink.packet_byte = uint8( (sink.sync_shift_reg << 1) | diff_enc_bit )
            sink.packet_byte_bit_count += 1
            sink.chip_shift_count = 0
            sink.chip_shift_reg   = zero( sink.chip_shift_reg )

            if sink.packet_byte_bit_count == 8
                sink.packetlen = sink.packet_byte
                println( "Packet lenght = ", sink.packetlen )
                set_state( sink, PayloadCollect )
                break
            end
        end
        
    end
end

function payloadcollect( sink::PacketSink{BPSK}, input::Vector )
    @printf( "PayloadCollect. input_idx: %d, chip_shift_reg: %s, chip_shift_count: %d, sync_shift_reg: %s\n", sink.input_idx, bin(sink.chip_shift_reg, 15),sink.chip_shift_count, hex(sink.sync_shift_reg,4))
    println( "PayloadCollect" )
    set_state( sink, SyncOnZero )    
end




function exec( sink::PacketSink{BPSK}, input::Vector )
    println( "exec" )

    sink.input_idx = 1

    while sink.input_idx <= length(input)

        @match sink.state begin
            SyncOnZero      => synconzero( sink, input )
            SFDSearch       => sfdsearch( sink, input )
            HeaderSearch    => headersearch( sink, input )
            PayloadCollect  => payloadcollect( sink, input )
        end

    end
end

type PacketSource
    modulation        # BPSK, OQPSK, etc
    chips_per_symbol  # how many chips per demodulated symbol. 15 For BPSK, 32 for OQPSK
    chip_map          # Chip to bit(s) mapping for modulation T
end

function PacketSource( ::Type{BPSK} )
    PacketSource( BPSK, 15, CHIP_MAP_BPSK )
end


function spread( chip_map::AbstractVector, chips_per_symbol::Integer, packet::AbstractVector{Uint8}; diff_enc = false )

    packet_len        = length( packet )
    frame_len         = packet_len * 8 * chips_per_symbol
    frame             = zeros( Uint8, frame_len )
    last_diff_enc_bit = 0
    frame_idx         = 0

    for packet_idx in 0:packet_len-1
        packet_byte = packet[packet_idx+1]

        for bit_idx in 0:7
            packet_bit = (packet_byte >> bit_idx) & 0x01

            if diff_enc
                packet_bit        = packet_bit $ last_diff_enc_bit
                last_diff_enc_bit = packet_bit
            end

            chips = chip_map[packet_bit+1]

            for chip_idx in 0:chips_per_symbol-1
                frame[frame_idx+1] = (chips >> chip_idx) & 0x01
                frame_idx       += 1
            end

        end
    end


    return frame
end


function make_packet( source::PacketSource, input::Vector{Uint8} )
    payload_len = uint8( length( input ) & 0b0111111 )
    packet      = [ 0x00, 0x00, 0x00, 0x00, 0xA7, payload_len, input ]
end

function exec( source::PacketSource, input::Vector{Uint8} )
    packet = make_packet( source, input )
    spread( CHIP_MAP_BPSK, source.chips_per_symbol, packet, diff_enc = false )
end



end # module PacketSink


using ieee802_15_4

source          = PacketSource( BPSK )
sink            = PacketSink( BPSK, 0 )
tx_packet_bytes = make_packet( source, [0xFF] )
# tx_packet_bytes = [ rand( Uint8, 32 ), tx_packet_bytes ]
tx_packet_chips = spread( CHIP_MAP_BPSK, source.chips_per_symbol, tx_packet_bytes, diff_enc = false )

rx_packet       = exec( sink, tx_packet_chips )
