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

var bytes = [0x04, 0x00, 0xa6, 0x09, 0x22, 0x0f, 0xef, 0x26, 00, 00, 00, 00]

function Decode(fport, bytes, variables){
    var AQI = returnInt(bytes, 8);
    var CO2 = returnInt(bytes, 10);
    return{
        "id": returnInt(bytes, 0),
        "temp": intToInt(bytes, 2)/100,
        "hum": returnInt(bytes, 4)/100,
        "press": returnInt(bytes, 6)/100,
        "AQI": AQI == 0 ? 'nan' : AQI/10,
        "CO2": CO2 == 0 ? 'nan' : CO2,
    };
}

/*var obj = Decode(0, bytes, 0);
console.log(obj['id']);
console.log(obj['temp']);
console.log(obj['hum']);
console.log(obj['press']);
console.log(obj['AQI']);
console.log(obj['CO2']);*/

var obj = {'interval': 5}

function intToBytes(integer) {
    var b = [];
    while (integer > 0) {
      b.unshift(integer & 0xff);
      integer >>= 8;
    }
    return b.reverse();
}
  
function Encode(fPort, obj, variables) {
    for(var v in obj){
        if(v === 'interval' && obj[v] > 0){
            return intToBytes(obj[v])
        }
    }
    return [];
}
console.log(Encode(0, obj, 0))