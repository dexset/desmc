module des.mc.calibrate.util;

public import des.math.linear;

/+ expected value: мат. ожидание +/
@property fSeg expected( in fSeg[] buf )
{
    fSeg mm;
    foreach( r; buf ) mm += r;
    return mm / cast(float)buf.length;
}
unittest
{
    auto r1 = fSeg( vec3(1,2,3), vec3(2,3,4) );
    auto r2 = fSeg( vec3(4,5,6), vec3(5,2,3) );
    auto r3 = fSeg( vec3(2,3,4), vec3(1,2,3) );
    auto re = ( r1 + r2 + r3 ) / 3.0f;
    assert( [r1,r2,r3].expected == re );
}

/+ variance: дисперсия +/
@property fSeg variance( in fSeg[] buf, in fSeg exp )
{
    vec3 Ds;
    vec3 De;
    foreach( r; buf )
    {
        Ds += (r.start - exp.start) ^^ 2;
        De += (r.dir - exp.dir) ^^ 2;
    }
    float lm1 = buf.length - 1;
    return fSeg( Ds / lm1 , De / lm1 );
}

fSeg approxIntersection( in fSeg s1, in fSeg s2 )
{ return s1.altitude( s2 ); }

unittest
{
    auto r1 = fSeg( vec3(0,0,.1), vec3(1,1,0) );
    auto r2 = fSeg( vec3(4,0,-0.1), vec3(-4,4,0) );
    auto res = approxIntersection( r1, r2 );
    auto ctr = res.start + res.dir/2;
    assert( ctr == vec3( 2, 2, 0 ) );
}

unittest
{
    auto r1 = fSeg( vec3(0,0,0), vec3(1,1,0) );
    auto r2 = fSeg( vec3(1,0,-1), vec3(-4,4,0) );
    auto res = approxIntersection( r1, r2 );
    auto ctr = res.start + res.dir/2;
    assert( ctr == vec3( 0.5, 0.5, -.5 ) );
}

version(unittest)
{
    private import des.math.method.stat.randn;
    private auto v = [ 
        -0.779237,  0.095773,  0.794196,  0.037817,  0.243846, -0.610992,  
         0.894963,  0.760108,  0.456166,  0.091022,  0.544440, -0.104452,  
         0.602444,  0.622659,  0.310276, -0.838316, -0.174904,  0.720212, 
        -0.587296, -0.140814,  0.752186, -0.679231,  0.108245, -0.476829, 
        -0.149560,  0.488450, -0.338691, -0.337862,  0.182438,  0.430244, 
        -0.246279, -0.489203, -0.204999,  0.804444, -0.903342, -0.842680,  
         0.638648,  0.392753, -0.150701, -0.259307, -0.471528, -0.927970, 
         0.053038,  0.866550, -0.262863, -0.015059,  0.797605, -0.378268, 
        -0.768152, -0.966215,  0.358892, -0.506356, -0.174640,  0.744792,  
         0.136404,  0.877210, -0.148166, -0.737943, -0.677909,  0.407338,  
         0.940582, -0.825226, -0.435503,  0.965236,  0.129071,  0.185805, 
        -0.323059, -0.055682, -0.077424,  0.583382, -0.000693,  0.315968, 
        -0.336294, -0.734126, -0.048058,  0.100198,  0.582619, -0.704076, 
        -0.658022, -0.083976, -0.179074,  0.304228,  0.150274, -0.428972,  
         0.005536,  0.600618,  0.158177,  0.511973, -0.834075,  0.147372, 
         0.831916, -0.915125, -0.211189, -0.921590, -0.186989,  0.628901, 
        -0.726222, -0.735542,  0.941152, -0.978562, -0.124824,  0.193818 ];
    private static size_t n;
    float rnd() { return v[n++%$]; }
    auto rndVec3() { return vec3( rnd(), rnd(), rnd() ); }
    fSeg rndSeg() { return fSeg( rndVec3() * 0.04, rndVec3() * 0.06 ); }
}
