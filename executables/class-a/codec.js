function uintToInt(uint, nbit) {
    nbit = +nbit || 32;
    if (nbit > 32) throw new RangeError('uintToInt only supports ints up to 32 bits');
    uint <<= 32 - nbit;
    uint >>= 32 - nbit;
    return uint;
}

function returnInt(bytes, idx){
    var num = bytes[idx] | bytes[idx+1] << 8;
    return uintToInt(num, 16);
}

function intToInt(bytes, idx){
    var sign = bytes[idx+1] & (1 << 7);
    var x = ((bytes[idx] & 0xFF) | ((bytes[idx+1] & 0xFF) << 8));
    if(sign) return 0xFFFF0000 | x;
    else return x;
}


var bytes = [0x04, 0x00, 0x6c, 0x09, 0xdb, 0x0e, 0xf0, 0x26, 0x00, 0x00, 0x00, 0x00]

function Decode(fport, bytes, variables){
    return{
        "id": returnInt(bytes, 0),
        "temp": intToInt(bytes, 2),
        "hum": returnInt(bytes, 4),
        "press": returnInt(bytes, 6),
        "AQI": returnInt(bytes, 8),
        "CO2": returnInt(bytes, 10),
    };
}
var obj = Decode(0, bytes, 0);
console.log(obj['id']);
console.log(obj['temp']);
console.log(obj['hum']);
console.log(obj['press']);
console.log(obj['AQI']);
console.log(obj['CO2']);


