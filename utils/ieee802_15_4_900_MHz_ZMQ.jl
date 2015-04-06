using ZMQ

function main()
    ctx          = Context( 1 )   
    zsock        = Socket( ctx, PULL )   
    packet_count = 0   
    connect( zsock, "tcp://localhost:5555" )
    
    while true
        msg = recv( zsock )
        out = convert( IOStream, msg )
        bytes = takebuf_array( out )
        packet_count += 1
        
        print( dec(packet_count,3), ": ")
        for i in 1:length(bytes)
            print( hex(bytes[i], 2), " " )
        end
        println()
    end
end

main()