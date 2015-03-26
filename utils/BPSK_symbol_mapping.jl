bitToChips  = [ 0b000100110101111, 0b111011001010000 ] # In hex: [ 0x09af, 0x7650 ]
D = 15
M = 2

function bitn( num, n )
    (num >> (n-1)) & 1
end

function symbolmap()
    symMap = Array( String, M * D )
    for n in 0:1, d in 0:D-1
        bit = bitn( bitToChips[ n+1 ], d+1  )
        idx = n*(D)+d
        # println( bit, " ", idx+1 )
        symMap[ idx + 1 ] = bit == 1 ? "1+0j" : "-1+0j"
    end
    
    symMap
end

symbols = symbolmap()


symbolString = "[ "

for i in 1:length(symbols)
    symbolString *= string( "(", symbols[i], ")" )
    if i != length(symbols)
        symbolString *= ", "
    else
        symbolString *= " ]"
    end
end

clipboard( symbolString )