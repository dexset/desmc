module desmc.multitrack.model.util;
import std.algorithm;
import std.array;

import desmc.core;

T[] plainArray(T)( in T[][] arr )
{
    T[] ret;
    return array( reduce!((r,a)=>(r~=a))( ret, arr ) );
}

class ClassifierClass
{
    float min_quality = 0.5;

    Skeleton mean;
    Skeleton[] array;

    float diff( in Skeleton s )
    {
        auto mj = mean.allJoints();
        auto sj = s.allJoints();

        auto hiq = getHighQualityIndexes( mj, sj );
        auto offset = calcOffsetInIndexes( mj, sj, hiq );
        auto dp = calcDistibution( offset );

        auto res = dp[0].len2 + dp[1].len;
        res = (res!=res) ? float.max : res;

        return res;
    }

    void append( in Skeleton s )
    {
        auto mj = mean.allJoints();
        auto sj = s.allJoints();

        if( array.length )
        {
            auto hiq = getHighQualityIndexes( mj, sj );
            auto offset = calcOffsetInIndexes( mj, sj, hiq );
            auto dp = calcDistibution( offset );

            auto n = array.length;

            foreach( ind; hiq )
            {
                mj[ind].pos = ( ( mj[ind].pos * n ) + sj[ind].pos ) / (n + 1);
                mj[ind].qual = 1.0f;
            }
        }
        else mean = s;

        array ~= s;
    }

    size_t[] getHighQualityIndexes( in Joint[] a, in Joint[] b )
    {
        size_t[] res;
        foreach( i; 0 .. a.length )
            if( a[i].qual > min_quality && b[i].qual > min_quality )
                res ~= i;
        return res;
    }

    vec3[] calcOffsetInIndexes( in Joint[] a, in Joint[] b, size_t[] hiq )
    {
        vec3[] res;
        foreach( index; hiq )
            res ~= b[index].pos - a[index].pos;
        return res;
    }

    static vec3[2] calcDistibution( in vec3[] arr )
    {
        vec3 exp;
        vec3 var;

        foreach( v; arr ) exp += v;
        exp /= cast(float)arr.length;
        foreach( v; arr ) var += (exp - v)^^2;
        var /= cast(float)(arr.length - 1);
        return [ exp, var ];
    }
}

unittest
{
    auto a = getFakeSkeletons(vec3(0,0,.1),[vec3(0,0,0)])[0];
    auto b = getFakeSkeletons(vec3(0,0.1,0),[vec3(0,0,0)])[0];

    auto cf = new ClassifierClass;
    import std.math;
    assert( abs( cf.diff(a) - float.max ) < 1e-12 );
    cf.append( a );
    assert( cf.diff(b) > 0.0f && cf.diff(b) < 0.1 );
}
