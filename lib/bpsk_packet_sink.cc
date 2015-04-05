/*
 * Copyright 2004,2013 Free Software Foundation, Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * ( at your option ) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <ieee802_15_4/bpsk_packet_sink.h>
#include <gnuradio/io_signature.h>
#include <cstdio>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdexcept>
#include <cstring>
#include <gnuradio/blocks/count_bits.h>
#include <iostream>


using namespace gr::ieee802_15_4;

#define VERBOSITY           0                                   // How much debug information to print, from 0 (nothing) to 2 (almost everthing)
#define CHIPS_PER_SYMBOL    15                                  // the standard specifies 15 chips per differentially encoded bit for BPSK
#define CHIP_MASK           0x3FFE                              // 0b0011111111111110, ignore the first and last chip since it depends on the last chip, FIXME: we can store the last chip
#define MAX_PREAMBLE_COUNT  8*CHIPS_PER_SYMBOL+3                // Maximum valid number of zero symbols received in a row while waiting for SFD

static const unsigned int CHIP_MAPPING[]  = { 0x09af, 0x7650 }; // See IEEE Std 802.15.4-2003, 6.6.2.3 Bit-to-chip mapping
static const int          MAX_PKT_LEN     = 128 -  1;           // remove header and CRC
static const int          MAX_LQI_SAMPLES = 8;                  // Number of chip correlation samples to take

class bpsk_packet_sink_impl : public bpsk_packet_sink {
public:

void enter_search()
{
    if ( VERBOSITY > 0 )
        fprintf( stderr, "@ enter_search\n");

    d_state             = STATE_SYNC_SEARCH;
    d_shift_reg         = 0;
    d_preamble_cnt      = 0;
    d_chip_cnt          = 0;
    d_packet_byte       = 0;
    d_last_diff_enc_bit = 0;
}

void enter_have_sync()
{
    if ( VERBOSITY > 0 )
        fprintf( stderr, "@ enter_have_sync\n");

    d_state             = STATE_HAVE_SYNC;
    d_packetlen_cnt     = 0;
    d_packet_byte       = 0;
    d_packet_byte_bit_count = 0;

    // Link Quality Information
    d_lqi              = 0;
    d_lqi_sample_count = 0;
}

void enter_have_header( int payload_len )
{
    if ( VERBOSITY > 0 )
        fprintf( stderr, "@ enter_have_header ( payload_len = %d )\n", payload_len );

    d_state             = STATE_HAVE_HEADER;
    d_packetlen         = payload_len;
    d_payload_cnt       = 0;
    d_packet_byte       = 0;
    d_packet_byte_bit_count = 0;
}


unsigned char decode_chips( unsigned short chips ){
    int i;
    int best_match    = 0xFF;
    int min_threshold = CHIPS_PER_SYMBOL + 1; // Matching to CHIPS_PER_SYMBOL chips, could never have a error of CHIPS_PER_SYMBOL + 1 chips
    // int diff_dec_bit;
    // int diff_enc_bit;

    for( i=0; i<2; i++) {
        
        
        unsigned int threshold = gr::blocks::count_bits16(( chips & CHIP_MASK ) ^ ( CHIP_MAPPING[i] & CHIP_MASK ));

        if ( threshold < min_threshold ) {
            best_match    = i;
            min_threshold = threshold;
        }
    }

    if ( min_threshold < d_threshold ) {
        if ( VERBOSITY > 2 )
            fprintf( stderr, "Found sequence with %d errors at 0x%x\n", min_threshold, ( chips & 0x7FFE ) ^ ( CHIP_MAPPING[best_match] & 0x7FFE )), fflush( stderr );

        // LQI: Average number of chips correct * MAX_LQI_SAMPLES
        if ( d_lqi_sample_count < MAX_LQI_SAMPLES ) {
            d_lqi += CHIPS_PER_SYMBOL - min_threshold;
            d_lqi_sample_count++;
        }

        return (unsigned char) best_match & 0x01 ;
    }

    return 0xFF;
}


bpsk_packet_sink_impl( int threshold )
  : block ("bpsk_packet_sink",
           gr::io_signature::make( 1, 1, sizeof( unsigned char )),
           gr::io_signature::make( 0, 0, 0 )),
           d_threshold( threshold )
{
    d_sync_vector      = 0xA7;
    d_lqi              = 0;
    d_lqi_sample_count = 0;

    if ( VERBOSITY > 0 )
        fprintf( stderr, "syncvec: %x, threshold: %d\n", d_sync_vector, d_threshold ),fflush( stderr );

    enter_search();
    message_port_register_out( pmt::mp("out"));
}

~bpsk_packet_sink_impl()
{
}

int general_work(   int                         noutput,
                    gr_vector_int&              ninput_items,
                    gr_vector_const_void_star&  input_items,
                    gr_vector_void_star&        output_items )
{

    const unsigned char *inbuf       = ( const unsigned char*)input_items[0];
    int                 ninput       = ninput_items[0];
    int                 count        = 0;
    int                 i            = 0;
    int                 rx_bit       = 0;
    int                 diff_dec_bit = 0;                 

    if ( VERBOSITY > 0 )
        fprintf( stderr,">>> Entering state machine\n"),fflush( stderr );

    while( count < ninput ) {
        switch( d_state ) {

        case STATE_SYNC_SEARCH:    // Look for sync vector
            if ( VERBOSITY > 1 )
                fprintf( stderr,"SYNC Search, ninput=%d syncvec=%x\n", ninput, d_sync_vector ),fflush( stderr );

            while ( count < ninput ) {

                if( inbuf[count++] & 1 )
                    d_shift_reg = ( d_shift_reg << 1 ) | 1;
                else
                    d_shift_reg = d_shift_reg << 1;

                if( d_preamble_cnt > 0 ){
                    d_chip_cnt = d_chip_cnt+1;
                }

                // The first if block syncronizes to chip sequences.
                if( d_preamble_cnt == 0 ){
                    rx_bit = decode_chips( d_shift_reg );

                    if ( rx_bit == 0 ) {
                        if ( VERBOSITY > 1 )
                            fprintf( stderr,"Found 0 in chip sequence\n"), fflush( stderr );
                        d_preamble_cnt += 1;  // we found a 0 in the chip sequence
                    }
                } else {
                    // we found the first 0, thus we only have to do the calculation every 15 chips
                    if( d_chip_cnt == 15 ){
                        d_chip_cnt = 0;
                        
                        rx_bit = decode_chips( d_shift_reg );
                                                
                        if ( rx_bit == 0xFF ) {
                            enter_search();
                            break;
                        }
                        
                        diff_dec_bit        = rx_bit ^ d_last_diff_enc_bit;
                        d_last_diff_enc_bit = rx_bit;
                        rx_bit              = diff_dec_bit;
                        
                        if ( rx_bit == 0 ) {
                            d_preamble_cnt += 1;
                        }
                        
                        if ( d_preamble_cnt > MAX_PREAMBLE_COUNT ) {
                            enter_search();
                            break;
                        }
                        
                        d_packet_byte = (d_packet_byte >> 1) | (rx_bit <<7);
                        
                        if ( d_packet_byte == d_sync_vector ) {
                            enter_have_sync();
                            break;
                        }
                    }
                }
            }
            break;

        case STATE_HAVE_SYNC:
            if ( VERBOSITY > 0 )
                fprintf( stderr,"Header Search bitcnt=%d, header=0x%08x\n", d_headerbitlen_cnt, d_header ),
                fflush( stderr );

            while ( count < ninput ) {        // Decode the bytes one after another.
                if( inbuf[count++] )
                    d_shift_reg = ( d_shift_reg << 1 ) | 1;
                else
                    d_shift_reg = d_shift_reg << 1;

                d_chip_cnt = d_chip_cnt+1;

                if( d_chip_cnt == CHIPS_PER_SYMBOL ){

                    d_chip_cnt = 0;
                    rx_bit     = decode_chips( d_shift_reg );

                    if( rx_bit == 0xFF ){
                        if( VERBOSITY > 1 )
                            fprintf( stderr, "Found a not valid chip sequence! %u\n", d_shift_reg ), fflush( stderr );
                        enter_search();
                        break;
                    }
                    
                    diff_dec_bit        = rx_bit ^ d_last_diff_enc_bit;
                    d_last_diff_enc_bit = rx_bit;
                    rx_bit              = diff_dec_bit;
                    
                    d_packet_byte = (d_packet_byte >> 1) | (rx_bit << 7);
                    d_packet_byte_bit_count += 1;
                    
                    if( d_packet_byte_bit_count == 8 ){
                        // we have a complete byte which represents the frame length.
                        int frame_len = d_packet_byte;
                        if( frame_len <= MAX_PKT_LEN ){
                            enter_have_header( frame_len );
                        } else {
                            enter_search();
                        }
                        break;
                    }
                }
            }
            break;

        case STATE_HAVE_HEADER:
            if ( VERBOSITY > 0 )
                fprintf( stderr,"Packet Build count=%d, ninput=%d, packet_len=%d\n", count, ninput, d_packetlen ),fflush( stderr );

            while ( count < ninput ) {                       // shift bits into bytes of packet one at a time
                if( inbuf[count++] )
                    d_shift_reg = ( d_shift_reg << 1 ) | 1;
                else
                    d_shift_reg = d_shift_reg << 1;

                d_chip_cnt = ( d_chip_cnt+1 ) % CHIPS_PER_SYMBOL;

                if( d_chip_cnt == 0 ){
                    rx_bit = decode_chips( d_shift_reg );
                    if( rx_bit == 0xff ){                         // something is wrong. restart the search for a sync
                        if( VERBOSITY > 1 )
                            fprintf( stderr, "Found a not valid chip sequence! %u\n", d_shift_reg ), fflush( stderr );

                        enter_search();
                        break;
                    }
                    
                    diff_dec_bit        = rx_bit ^ d_last_diff_enc_bit;
                    d_last_diff_enc_bit = rx_bit;
                    rx_bit              = diff_dec_bit;
                    
                    d_packet_byte = (d_packet_byte >> 1) | (rx_bit << 7);
                    d_packet_byte_bit_count += 1;

                    if( d_packet_byte_bit_count == 8 ){
                                                           // we have a complete byte
                        if ( VERBOSITY > 0 )
                            fprintf( stderr, "packetcnt: %d, payloadcnt: %d, payload 0x%x, d_packet_byte_bit_count: %d\n", d_packetlen_cnt, d_payload_cnt, d_packet_byte, d_packet_byte_bit_count ), fflush( stderr );

                        d_packet[d_packetlen_cnt++] = d_packet_byte;
                        d_payload_cnt              += 1;
                        d_packet_byte_bit_count     = 0;

                        if ( d_payload_cnt >= d_packetlen ){ // packet is filled, including CRC. might do check later in here
                            unsigned int scaled_lqi = ( d_lqi / MAX_LQI_SAMPLES ) << 3;
                            unsigned char lqi       = ( scaled_lqi >= 256? 255 : scaled_lqi );

                            pmt::pmt_t meta         = pmt::make_dict();
                            meta                    = pmt::dict_add( meta, pmt::mp("lqi"), pmt::from_long( lqi ));

                            std::memcpy( buf, d_packet, d_packetlen_cnt );
                            pmt::pmt_t payload      = pmt::make_blob( buf, d_packetlen_cnt );

                            message_port_pub( pmt::mp("out"), pmt::cons( meta, payload ));


                            if( VERBOSITY > 0 )
                                fprintf( stderr, "Adding message of size %d to queue\n", d_packetlen_cnt );
                            enter_search();
                            break;
                        }
                    }
                }
            }

            break;

        default:
            assert( 0 );
            break;

        }
    }

    if( VERBOSITY > 0 )
        fprintf( stderr, "Samples Processed: %d\n", ninput_items[0]), fflush( stderr );

        consume( 0, ninput_items[0]);

    return 0;
}

private:
    enum {STATE_SYNC_SEARCH, STATE_HAVE_SYNC, STATE_HAVE_HEADER} d_state;

    unsigned int      d_sync_vector;           // 802.15.4 standard is 4x 0 bytes and 1x0xA7
    unsigned int      d_threshold;             // how many bits may be wrong in sync vector

    unsigned int      d_shift_reg;             // used to look for sync_vector
    int               d_preamble_cnt;          // count on where we are in preamble
    int               d_chip_cnt;              // counts the chips collected
    int               d_last_diff_enc_bit;     // previous not yet decoded bit, need to keep it to do differential decoding

    unsigned int      d_header;                // header bits
    int               d_headerbitlen_cnt;      // how many so far

    unsigned char     d_packet[MAX_PKT_LEN];   // assembled payload
    unsigned char     d_packet_byte;           // byte being assembled
    int               d_packet_byte_bit_count; // how many bits, out of 8, of d_packet_byte bits have we shifted in so far
    int               d_packetlen;             // length of packet
    int               d_packetlen_cnt;         // how many so far
    int               d_payload_cnt;           // how many bytes in payload

    unsigned int      d_lqi;                   // Link Quality Information
    unsigned int      d_lqi_sample_count;

    char              buf[256];                // FIXME:
};

bpsk_packet_sink::sptr bpsk_packet_sink::make( unsigned int threshold ) {
    return gnuradio::get_initial_sptr( new bpsk_packet_sink_impl( threshold ));
}
