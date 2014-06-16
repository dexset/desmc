module desmc.multitrack.model.simple;

import desmc.multitrack.model;
import std.algorithm;
import desmc.multitrack.model.util;

struct SimpleClassifierParams
{
    float min_point_quality=0.5;
    float class_offset_limit=100;
    float class_deviation_limit=200;
}

class SimpleClassifier : Classifier
{
    SimpleClassifierParams params;

    this( in SimpleClassifierParams scp = SimpleClassifierParams() )
    {
        params = scp;
    }

    Skeleton[][] opCall( in Skeleton[][] skel_arr )
    {
        auto red = plainArray( skel_arr );
        ClassifierClass[] classes;
        foreach( skel; red )
        {
            auto res = findClass( classes, skel );
            auto cur = processResult( classes, res[0], res[1] );
            cur.append( skel );
        }
        return getSkeletons( classes );
    }

    static auto findClass( ClassifierClass[] classes, in Skeleton skel )
    {
        ClassifierClass fnd;
        float[2] min_diff = [ float.max, float.max ];
        foreach( cls; classes )
        {
            auto df = cls.diff(skel);
            if( df[0] < min_diff[0] && df[1] < min_diff[1] )
            {
                min_diff = df;
                fnd = cls;
            }
        }
        return tuple( fnd, min_diff );
    }

    ClassifierClass processResult( ref ClassifierClass[] classes,
                                       ClassifierClass cls, float[2] diff )
    {
        auto ret = cls;
        if( cls is null || diff[0] > params.class_offset_limit ||
                           diff[1] > params.class_deviation_limit )
        {
            ret = newClassifierClass();
            classes ~= ret;
        }
        return ret;
    }

    auto newClassifierClass()
    {
        return new ClassifierClass( params.min_point_quality );
    }

    static auto getSkeletons( ClassifierClass[] classes )
    {
        Skeleton[][] ret;
        foreach( cls; classes )
            ret ~= cls.array;
        return ret;
    }
}

unittest
{
    auto tsc = new SimpleClassifier( SimpleClassifierParams(0.5,2) );

    Skeleton[][] by_tracker;
    by_tracker ~= getFakeSkeletons(vec3(0,0,.1));
    by_tracker ~= getFakeSkeletons(vec3(0,0.1,0));

    auto by_group = tsc( by_tracker );

    //printSkeletonsDArray( by_tracker );
    //printSkeletonsDArray( by_group );

    assert( by_group[0][0] == by_tracker[0][0] );
    assert( by_group[0][1] == by_tracker[1][0] );
    assert( by_group[1][0] == by_tracker[0][1] );
    assert( by_group[1][1] == by_tracker[1][1] );
}

struct SimpleComplexerParams
{
    // TODO: params 
}

class SimpleComplexer : Complexer
{
    SimpleComplexerParams params;

    this( in SimpleComplexerParams scp = SimpleComplexerParams() )
    {
        params = scp;
    }

    Skeleton[] opCall( in Skeleton[][] skels )
    {
        auto min_qual = 0.5;
        Skeleton[] result;
        foreach( group; skels )
        {
            if( group.length == 0 ) continue;

            Skeleton mean = group[0];
            if( group.length < 2 ) { result ~= mean; continue; }
            size_t n = 1;
            auto mj = mean.allJoints();
            foreach( s; group[1 .. $] )
            {
                auto sj = s.allJoints();
                auto hiq = new ubyte[]( sj.length );
                foreach( i, ref h; hiq )
                    h = (mj[i].qual > min_qual)*2 + (sj[i].qual > min_qual);
                auto offset = new vec3[]( sj.length );
                vec3 offset_exp;
                size_t offset_exp_cnt;
                foreach( i, h; hiq )
                    if( h == 3 )
                    {
                        auto buf = sj[i].pos - mj[i].pos;
                        offset[i] = buf;
                        offset_exp += buf;
                        offset_exp_cnt++;
                    }
                offset_exp /= cast(float)offset_exp_cnt;
                foreach( i, h; hiq )
                {
                    if( h == 3 )
                    {
                        mj[i].pos = mj[i].pos + offset[i] / ( 1.0f + n );
                        mj[i].qual = 1.0f;
                    }
                    else if( h == 2 )
                    {
                        mj[i].qual = 0.75f;
                    }
                    else if( h == 1 )
                    {
                        mj[i].pos = sj[i].pos;
                        mj[i].qual = 0.5f;
                    }
                    else
                    {
                        mj[i].qual = 0.0f;
                    }
                }
                n++;
                mean.setJoints( mj );
            }
            result ~= mean;
        }
        return result;
    }
}

unittest
{
    auto tsc = new SimpleComplexer;

    auto us0 = getFakeSkeletons(vec3(0,0,0));
    auto us1 = getFakeSkeletons(vec3(0,0,.1));
    auto us2 = getFakeSkeletons(vec3(0,0,-.1));
    auto by_group = [ [ us1[0], us2[0] ],
                      [ us1[1], us2[1] ] ];

    auto cmpl = tsc( by_group );
    assert( us0 == cmpl );
}

struct SimpleUserHandlerParams
{
    float max_transform_dist;
}

class SimpleUserHandler: UserHandler
{
protected:
    bool is_overdue = true;
    User self_user;
    SimpleUserHandlerParams params;
public:

    this( User fuser, in SimpleUserHandlerParams suhp )
    {
        self_user = fuser;
        is_overdue = false;
        params = suhp;
    }

    @property 
    {
        bool updated() const { return !is_overdue; }
        ref const(User) user() const { return self_user; }
        bool isOverdue() const { return is_overdue; }
    }

    void setOverdue() { is_overdue = true; }

    void setSkeleton( in Skeleton s )
    {
        self_user.skel = s;
        is_overdue = false;
    }

    float calcTransformPossibility( in Skeleton s ) const
    {
        auto max_dist2 = params.max_transform_dist ^^ 2;
        auto dist2 = (self_user.skel.torso.pos - s.torso.pos).len2;
        if( dist2 > max_dist2 ) return 0.0f;
        return 1.0f / ( dist2 + 0.0001f );
    }
}

unittest
{
    auto us0 = getFakeSkeletons(vec3(0,0,0),[vec3(0,0,0)])[0];
    auto tsuh = new SimpleUserHandler( User(0,us0), SimpleUserHandlerParams(1.0f) );
    assert( tsuh.updated );
    assert( tsuh.user == User(0,us0) );
    assert( !tsuh.isOverdue );

    assert( tsuh.calcTransformPossibility(us0) >= 0.9f / 0.0001f );
    assert( tsuh.calcTransformPossibility(skeleton_offset(us0,vec3(1.1,0,0))) == 0.0f );
    auto tctpn = tsuh.calcTransformPossibility(skeleton_offset(us0,vec3(0.5,0,0)));
    assert( tctpn > 0.0f );
    assert( tctpn < 1.0f / 0.0001f );
}

struct SimpleDestributorParams
{
    // TODO: params
}

class SimpleDestributor : Destributor
{
    SimpleDestributorParams params;

    this( in SimpleDestributorParams sdp = SimpleDestributorParams() )
    {
        params = sdp;
    }

    Skeleton[] opCall( UserHandler[] handlers, in Skeleton[] skeletons )
    {
        auto table = calcPossibility( handlers, skeletons );
        bool[] destributed, updated;
        destributed.length = skeletons.length;
        updated.length = handlers.length;

        foreach( k; 0 .. min( skeletons.length, handlers.length) )
        {
            float max_possibility = 0;
            ptrdiff_t max_i = -1;
            ptrdiff_t max_j = -1;

            foreach( i, skel_line; table )
            {
                if( destributed[i] ) continue;
                foreach( j, coef; skel_line )
                {
                    if( updated[j] ) continue;

                    if( coef > max_possibility )
                    {
                        max_possibility = coef;
                        max_i = i;
                        max_j = j;
                    }
                }
            }

            if( max_i >= 0 )
            {
                handlers[max_j].setSkeleton( skeletons[max_i] );
                updated[max_j] = true;
                destributed[max_i] = true;
            }
        }

        Skeleton[] not_destributed;
        foreach( i, skel; skeletons )
            if( !destributed[i] )
                not_destributed ~= skel;
        return not_destributed;
    }

protected:

    float[][] calcPossibility( UserHandler[] handlers, in Skeleton[] skeletons )
    {
        float[][] ret;
        foreach( skel; skeletons )
        {
            float[] buf;
            foreach( uh; handlers )
                buf ~= uh.calcTransformPossibility( skel );
            ret ~= buf;
        }
        return ret;
    }
}

unittest
{
    auto skels0 = getFakeSkeletons(vec3(0,0,0));
    auto tsd = new SimpleDestributor;
    assert( skels0 == tsd([],skels0) );
    UserHandler[] uhlist;
    foreach( i, s; skels0 )
        uhlist ~= new SimpleUserHandler( User(i,s), SimpleUserHandlerParams(1.0f) );
    auto skels1 = getFakeSkeletons(vec3(0,0.2,0));
    assert( [] == tsd(uhlist,skels1) );
    import std.array;
    auto uhskels = array( map!(a=>a.user.skel)(uhlist) );
    assert( uhskels == skels1 );
}

struct SimpleMultiTrackerFactoryParams
{
    SimpleClassifierParams classifier;
    SimpleComplexerParams complexer;
    SimpleDestributorParams destributor;
    SimpleUserHandlerParams user;
}

class SimpleMultiTrackerFactory : MultiTrackerFactory
{
protected:
    Classifier _classifier;
    Complexer _complexer;
    Destributor _destributor;

    float max_user_transform_dist = 1.0f;

    SimpleMultiTrackerFactoryParams params;

public:
    this( in SimpleMultiTrackerFactoryParams smtfp )
    {
        params = smtfp;
        _classifier = new SimpleClassifier( params.classifier );
        _complexer = new SimpleComplexer( params.complexer );
        _destributor = new SimpleDestributor( params.destributor );
    }

    @property
    {
        Classifier classifier() { return _classifier; }
        Complexer complexer() { return _complexer; }
        Destributor destributor() { return _destributor; }
    }

    UserHandler newUserHandler( User user )
    { return new SimpleUserHandler( user, params.user ); }
}

version(unittest)
{
    static void printSkeletonsArray( Skeleton[] arr )
    {
        import std.stdio;
        write( "[ " );
        foreach( j, sk; arr )
            writef( "Skeleton#%d torso: %s ", j, sk.torso.pos.data );
        writeln( " ]" );
    }

    static void printSkeletonsDArray( Skeleton[][] arr )
    {
        import std.stdio;
        if( arr.length == 0 ) 
        {
            writeln( "empty array" );
            return;
        }
        writeln( "[ --------- " );
        foreach( i, list; arr )
            printSkeletonsArray( list );
        writeln( "  --------- ]" );
    }
}
