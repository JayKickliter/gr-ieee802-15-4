using ieee802_15_4

source          = PacketSource( BPSK )
sink            = PacketSink( BPSK, 0, diff_enc = true )
payload         = [ 0xDE, 0xAD, 0xFE, 0xED ]
tx_packet_chips = exec( source, payload, diff_enc = true )


exec( sink, tx_packet_chips )

exec( sink, tx_packet_chips[51:end-20] )
exec( sink, tx_packet_chips[1:50] )
exec( sink, tx_packet_chips[51:end-20] )
exec( sink, tx_packet_chips[end-19:end] )

