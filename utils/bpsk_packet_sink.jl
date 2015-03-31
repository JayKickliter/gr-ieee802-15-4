module ieee802_15_4

using Match


export  PacketSink,
        PacketSource,
        Modulation,
        BPSK,
        OQPSK,
        spread,
        CHIP_MAP_BPSK,
        exec


abstract SyncState
type SyncSearch <: SyncState end
type HeaderSearch   <: SyncState end
type PayloadCollect <: SyncState end


abstract Modulation
type BPSK <: Modulation end
type OQPSK <: Modulation end

const STATE_SYNC_SEARCH = 0
const STATE_HAVE_SYNC   = 1
const STATE_HAVE_HEADER = 2
const MAX_PKT_LEN       = 127
const CHIP_MAP_BPSK     = Uint16[ 0b000100110101111, 0b111011001010000 ]
const CHIP_MASK_BPSK    = 0b0011111111111110



type PacketSink
    modulation        # BPSK, OQPSK, etc
    state             # what is our SyncState
    chips_per_symbol  # how many chips per demodulated symbol. 15 For BPSK, 32 for OQPSK
    chip_map          # Chip to bit(s) mapping for modulation T
    sync_sequence     # 802.15.4 standard is 4x 0 bytes and 1x0xA7, we will ignore the first byte
    threshold         # how many bits may be wrong in sync vector
    sync_shift_reg    # decoded chips are shifted in, and compared against sync_sequence
    sync_shift_count  # how many bits have we shifted into sync_shift_reg
    chip_shift_reg    # chips are shifted in and decoded to look for first 0
    chip_shift_count  # how many chips have we shifted into chip_shift_reg
    last_diff_enc_bit # previous not-yet-decoded bit, need to keep it to do differential decoding
    packet            # assembled payload
    packet_byte       # byte being assembled
    packet_byte_count   # which bit of d_packet_byte we're working on
    packetlen         # length of packet
    packetlen_cnt     # how many so far
    payload_cnt       # how many bytes in payload
    input_idx         # our location in the input vector
end

function PacketSink( modType, threshold )
    modulation        = BPSK
    state             = SyncSearch
    chips_per_symbol  = 15
    chip_map          = CHIP_MAP_BPSK
    sync_sequence     = 0x00e5
    threshold         = threshold
    sync_shift_reg    = zero( Uint16 )
    sync_shift_count  = 0
    chip_shift_reg    = zero( Uint16 )
    chip_shift_count  = 0
    last_diff_enc_bit = 0
    packet            = zeros( Uint8, MAX_PKT_LEN )
    packet_byte       = zero( Uint8 )
    packet_byte_count = 0
    packetlen         = 0
    packetlen_cnt     = 0
    payload_cnt       = 0
    input_idx         = 0

    PacketSink( modulation, state, chips_per_symbol, chip_map, sync_sequence, threshold, sync_shift_reg, sync_shift_count, chip_shift_reg, chip_shift_count, last_diff_enc_bit, packet, packet_byte, packet_byte_count, packetlen, packetlen_cnt, payload_cnt, input_idx )

end


function count_bit_diffs( a, b, mask )
    count_ones( (a & mask) $ (b & mask) )
end

function demap_chips( sink::PacketSink, chips::Integer )
    # println( "demap_chips" )
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

    is_valid_seq = min_errors < sink.threshold

    return ( is_valid_seq, best_match )
end


function set_state( sink::PacketSink, ::Type{SyncSearch} )
    println( sink.state, " -> SyncSearch" )
    sink.state            = STATE_SYNC_SEARCH
    sink.last_diff_enc_bit = 0
    sink.sync_shift_reg   = zero( Uint16 )
    sink.sync_shift_count = 0
    sink.chip_shift_reg   = zero( Uint16 )
    sink.chip_shift_count = 0
    sink.packet_byte      = 0
end

function set_state( sink::PacketSink, ::Type{HeaderSearch} )
    println( sink.state, " -> HeaderSearch" )
    sink.state           = STATE_HAVE_SYNC
    sink.packet_byte     = 0
    sink.packet_byte_count = 0
end

function set_state( sink::PacketSink, ::Type{PayloadCollect} )
    println( sink.state, " -> "PayloadCollect )
    sink.state           = PayloadCollect
    sink.packetlen       = payload_len
    sink.payload_cnt     = 0
    sink.packet_byte     = 0
    sink.packet_byte_count = 0
end

function syncsearch( sink::PacketSink, input::Vector )
    while sink.input_idx <= length( input )

        # Shift chips into the register, to look for first zero
        sink.chip_shift_reg    = uint16( (sink.chip_shift_reg >> 1) | ((input[sink.input_idx] & 1)<<14) )
        @printf( "syncsearch. input_idx: %d, chip_shift_reg: %s, chip_shift_count: %d, sync_shift_reg: %s sync_shift_count: %d\n", sink.input_idx, bin(sink.chip_shift_reg, 15),sink.chip_shift_count, hex(sink.sync_shift_reg,4), sink.sync_shift_count )        
        sink.input_idx        += 1
        sink.chip_shift_count += 1


        # Compare shift register against the chip sequence representing 0
        # if it is a zero, begin processing 15 chip sequences at a time
        # and shift those into decoded bits into the sync_shift_reg to look for the
        # sync_sequence

        # Have not yet shifted first bit into sync_shift_reg
        if sink.sync_shift_count == 0
            (is_valid_seq, diff_enc_bit) = demap_chips( sink, sink.chip_shift_reg )
            diff_dec_bit                 = diff_enc_bit $ sink.last_diff_enc_bit
            if is_valid_seq && diff_enc_bit == 0
                println( "have first zero" )
                sink.last_diff_enc_bit = diff_enc_bit
                sink.sync_shift_reg = (sink.sync_shift_reg << 1) | diff_enc_bit
                # sink.sync_shift_reg = sink.sync_shift_reg << 1 | diff_dec_bit
                sink.sync_shift_count += 1
                sink.chip_shift_count = 0
                sink.chip_shift_reg   = zero( sink.chip_shift_reg )
            end # if is_valid_seq && diff_dec_bit == 0
        else
            if sink.sync_shift_count < 16
                if sink.chip_shift_count == sink.chips_per_symbol
                    (is_valid_seq, diff_enc_bit) = demap_chips( sink, sink.chip_shift_reg )
                    diff_dec_bit                 = diff_enc_bit - sink.last_diff_enc_bit
                    if is_valid_seq
                        println( "168" )
                        sink.sync_shift_reg = (sink.sync_shift_reg << 1) | diff_enc_bit
                        # sink.sync_shift_reg = sink.sync_shift_reg << 1 | diff_dec_bit
                        sink.sync_shift_count += 1
                        sink.chip_shift_count = 0
                        sink.chip_shift_reg   = zero( sink.chip_shift_reg )
                    else
                        println( "175" )
                        set_state( sink, SyncSearch )
                    end # if is_valid_seq
                end 
            else
                    println( "180: checking sync sequence ", hex(sink.sync_shift_reg) )
                    if sink.sync_shift_reg == sink.sync_sequence
                        println( "182: got a sync sequence at input[&(sink.input_idx)]" )
                        set_state( sink, HeaderSearch )
                        break
                    else
                        println("186")
                        set_state( sink, SyncSearch )
                    end # if sink.sync_shift_reg == sink.sync_sequence
            end
        end
    end # sink.input_idx <= length( input )

end


function headersearch( sink::PacketSink, input::Vector )
    println( "headersearch" )

end

function payloadcollect( sink::PacketSink, input::Vector )
    println( "PayloadCollect" )
end




function exec( sink::PacketSink, input::Vector )
    println( "exec" )

    sink.input_idx = 1

    while sink.input_idx <= length(input)

        @match sink.state begin
            SyncSearch      => syncsearch( sink, input )
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



function exec( source::PacketSource, input::Vector{Uint8} )
    payload_len = uint8( length( input ) & 0b0111111 )
    packet      = [ 0x00, 0x00, 0x00, 0xA7, payload_len, input ]
    
    spread( CHIP_MAP_BPSK, source.chips_per_symbol, packet, diff_enc = false )
    
end



end # module PacketSink


using ieee802_15_4

source    = PacketSource( BPSK )
tx_packet = exec( source, [0x00])
#111010110010001

sink      = PacketSink( BPSK, 1 )
rx_packet = exec( sink, tx_packet )


bin(0xA7,8)

"00000000"

# A7 "10100111"
# A7 Rev "11100101"
# 0x00 rev with A7 rev 0b0000000011100101
# 0x00e5