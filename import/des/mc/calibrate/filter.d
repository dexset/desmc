module des.mc.calibrate.filter;

import des.mc.calibrate.util;

class FilterBuffer(T)
    if( is( typeof( (T[]).init.expected ) == T ) && is( typeof( T[].init.variance(T.init) ) == T ) )
{
private:
    size_t max_length;
    size_t current_index = 0;
    T[] data;

public:

    this( size_t maxLen )
    {
        assert( maxLen > 0 );
        max_length = maxLen;
    }

    @property
    {
        bool isFilled() const { return data.length >= max_length; }
        float fillRatio() const { return data.length / cast(float)max_length; }

        T[2] distributionParams() const
        {
            auto exp = data.expected;
            auto var = data.variance(exp);
            return [ exp, var ];
        }

        size_t maxLength() const { return max_length; }
        size_t maxLength( size_t maxLen )
        {
            max_length = maxLen;
            return max_length;
        }
    }

    void reset()
    {
        data.length = 0;
        current_index = 0;
    }

    void append( in T nval )
    {
        if( isFilled ) data[current_index%$] = nval;
        else data ~= nval;
        current_index = (current_index+1) % max_length;
    }
}

