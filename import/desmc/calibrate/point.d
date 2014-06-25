module desmc.calibrate.point;

public import std.math;
public import std.algorithm;
public import std.array;
public import desmath.linear;

import desmath.basic.traits;

import desmc.calibrate.filter;
public import desmc.calibrate.util;

class CalibratorException : Exception
{
    this( string msg, string file=__FILE__, size_t line=__LINE__ ) 
        @safe pure nothrow
    { super( msg, file, line ); }
}

enum bad_vec3 = vec3( float.nan, float.nan, float.nan );
 
struct PointCalibrationResult
{
    enum State
    {
        BUFFERED,
        ACCEPTED,
        ABORTED
    }

    State state;
    fSeg avg_ray;
    float quality;
    float stability;
    vec3 point = bad_vec3;

    @property bool done() const { return !!point; }
}

struct PointCalibratorParam
{
    size_t filter_length = 100;
    float deviation_limit;
    float min_quality = .5f;
    float min_stability = .9f;
}

class PointCalibrator
{
protected:
    PointCalibratorParam params;

    FilterBuffer!fSeg fbuffer;
    fSeg[] result_ray;

    vec3 last_point = bad_vec3;

public:

    this( PointCalibratorParam pcp )
    {
        params = pcp;
        fbuffer = new FilterBuffer!fSeg( params.filter_length );
    }

    auto filter( in fSeg seg )
    {
        fbuffer.append(seg);
        auto dp = fbuffer.distributionParams;
        auto avg = dp[0];
        auto var = dp[1];

        auto quality = calcQuality( result_ray ~ avg );
        auto stability = calcStability( var );

        auto result_state = PointCalibrationResult.State.BUFFERED;

        if( checkStability(stability) )
        {
            if( checkQuality(quality) )
            {
                result_ray ~= avg;
                fbuffer.reset();
                result_state = PointCalibrationResult.State.ACCEPTED;
            }
            else result_state = PointCalibrationResult.State.ABORTED;
        }

        last_point = calcPoint();
        return PointCalibrationResult( result_state, avg, quality, 
                                        stability, calcPoint() );
    }

    @property auto point() const { return last_point; }

protected:

    float calcQuality( in fSeg[] list ) const
    {
        if( list.length < 2 ) return 0;
        float sum = 0;
        foreach( i; 0 .. list.length-1 )
            foreach( j; i+1 .. list.length )
            {
                auto a = list[i].dir.e;
                auto b = list[j].dir.e;
                auto alpha = acos(a^b);
                auto norm = (a * b).e;
                auto mv = list[i].start - list[j].start;
                sum += sin(alpha) / ( 1.0f + abs(norm ^ mv) );
            }
        return sum;
    }

    float calcStability( in fSeg variance ) const
    {
        auto p1 = normalizeDeviation( sqrt(variance.start.len) );
        auto p2 = normalizeDeviation( sqrt(variance.dir.len) );
        return ( 1.0f - ( p1 + p2 ) / 2.0f ) * fbuffer.fillRatio;
    }

    float normalizeDeviation( float deviation ) const
    {
        if( !isFinite(deviation) || deviation == 0 ) return 1;
        return max( (deviation-params.deviation_limit) / deviation, 0 );
    }

    bool checkQuality( float quality ) const
    { return quality > params.min_quality || !result_ray.length; }

    bool checkStability( float stability ) const
    { return stability > params.min_stability && fbuffer.isFilled; }

    bool isResultable() const { return result_ray.length > 1; }

    auto calcPoint() const
    {
        if( !isResultable ) return bad_vec3;

        vec3 res;
        ulong k = 0;

        foreach( i; 0 .. result_ray.length-1 )
            foreach( j; i+1 .. result_ray.length )
            {
                k++;
                auto ai = approxIntersection( result_ray[i], result_ray[j] );
                res += ai.start + ai.dir / 2.0f;
            }
        res /= k;

        return res;
    }
}

unittest
{
    auto pc = new PointCalibrator( PointCalibratorParam(20, 0.15) );

    auto r1 = fSeg( vec3(0,0,0), vec3(1,1,0) );
    auto r2 = fSeg( vec3(1,0,-1), vec3(-4,4,0) );

    PointCalibrationResult q;

    q = pc.filter( r1 + rndSeg() );

    assert( q.state == PointCalibrationResult.State.BUFFERED );

    while( q.state != PointCalibrationResult.State.ACCEPTED ) 
        q = pc.filter( r1 + rndSeg() );

    assert( !q.done );

    q.state = PointCalibrationResult.State.BUFFERED;

    while( q.state != PointCalibrationResult.State.ACCEPTED )
        q = pc.filter( r2 + rndSeg() );

    assert( q.point );
    assert( q.point == pc.point );
    assert( (pc.point - vec3(.5,.5,-.5)).len2 < 0.1 );
}

unittest
{
    auto pc = new PointCalibrator( PointCalibratorParam(20, 0.15) );
    auto r1 = fSeg( vec3(0,0,0), vec3(1,1,0) );
    PointCalibrationResult q;
    while( q.state != PointCalibrationResult.State.ACCEPTED ) 
        q = pc.filter( r1 + rndSeg() );

    while( q.state != PointCalibrationResult.State.ABORTED ) 
    {
        q = pc.filter( r1 + rndSeg() );
        assert( q.quality < 0.1 );
    }
    assert( q.state == PointCalibrationResult.State.ABORTED );
}
