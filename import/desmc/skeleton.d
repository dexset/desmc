import std.stdio;
import std.string;
import std.algorithm;
import desmath.linear;

struct Joint
{
    vec3 pos;
    float rel;
}

struct Skeleton
{
private:

    Joint[8] joints;

    enum JointID
    {
        HEAD,
        NECK,
        RIGHT_SHOULDER,
        RIGHT_ELBOW,
        RIGHT_HAND,
        LEFT_SHOULDER,
        LEFT_ELBOW,
        LEFT_HAND
    }

public:

    mixin( accessArray!("joints",Joint,JointID) );

}

private
{

@property string accessArray(string arrayName,arrayType,Enum)()
{
    string[] ret;

    string type = arrayType.stringof;

    foreach( e; Enum.min .. Enum.max )
    {
        string id = format("%s.%s",Enum.stringof,e);
        string name = toProperName( id );
        string access = format( "%s[%s]", arrayName, id );
        ret ~= format( "@property ref const(%1$s) %2$s() const { return %3$s; }\n" ~
                       "@property void %2$s( in %1$s v ) { %3$s = v; }",
                       type, name, access );
    }

    return ret.join("\n");
}

string toProperName( string str )
{
    auto words = str.split(".")[$-1].split("_");
    auto res = toLowerPure( words[0] );
    if( words.length > 1 )
        res = reduce!( (a,b) => a ~ b )( res, map!( a => a.capitalize )(words[1 .. $]) );
    return res;
}

unittest
{
    assert( toProperName( "RIGHT_HAND" ) == "rightHand" );
    assert( toProperName( "RIGHT_HAND_FIRST_FINGER" ) == "rightHandFirstFinger" );
    assert( toProperName( "HEAD" ) == "head" );
}

pure string toLowerPure( string str )
{
    auto diff = cast(ubyte)('A') - cast(ubyte)('a');
    char[] buf;
    auto upperStart = cast(ubyte)('A');
    auto upperEnd = cast(ubyte)('Z');
    foreach( c; str )
    {
        auto bc = cast(ubyte)(c);
        if( bc >= upperStart && bc <= upperEnd )
            buf ~= cast(char)(bc-diff);
        else
            buf ~= c;
    }

    return buf.idup;
}

unittest
{
    assert( toLowerPure( "OlolOlo" ) == "olololo" );
    assert( toLowerPure( "RIGHT" ) == "right" );
    assert( toLowerPure( "r10%Z" ) == "r10%z" );
    string rndstr = "`1234567890-=qwertyuiop[]\\asdfghjkl;'zxcvbnm,./~!@#$%^&*()_+QWERTYUIOP{}|ASDFGHJKL:\"ZXCVBNM<>?";
    assert( toLowerPure( rndstr ) == rndstr.toLower );
}

}
