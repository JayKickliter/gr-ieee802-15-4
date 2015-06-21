# Data flow
#   [bytes] -> [bits (LSB first)] -> [differential encoding] -> [chip mapping (LSB first)]  -> [symbol mapping]

#                  0  1
bpsk_chip_mask = [ 1  0  
                   1  0  
                   1  0  
                   0  1  
                   1  0  
                   1  0  
                   0  1  
                   0  1  
                   1  0  
                   0  1  
                   1  0  
                   0  1  
                   0  1  
                   0  1  
                   0  1 ]  

xor(a,b) = a$b

function bytes2chips(data::AbstractVector{Uint8})
    nBits  = 8
    outLen = nBits * length(data) * 15
    out    = Array(Int, outLen)
    outIdx = 1
    encBit = 0
    
    for element in data
        for bitIdx in 0:nBits-1
            bitVal = (element >> bitIdx) & 1
            encBit = xor(encBit, bitVal) 
            chipSeq = bpsk_chip_mask[:,encBit+1]
            for chip in chipSeq
                out[outIdx] = chip
                outIdx += 1
            end
        end
    end
    
    out
end

function bit2symbol(bit)
    (1,-1)[bit+1]
end

function bit2symbol(bits::AbstractVector)
    [bit2symbol(bit) for bit in bits]
end

chips   = bytes2chips([0x00, 0x00, 0x00,0xA7])

symbols = bit2symbol(chips)

clipboard(symbols)

# out[n D + k] = symbol_table[in[n] D + k], k=0,1,...,D-1
