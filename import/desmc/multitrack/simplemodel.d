module desmc.multitrack.simplemodel;

import desmc.multitrack.model;
import std.algorithm;

T[] plainArray(T)( in T[][] arr )
{
    T[] ret;
    foreach( p; arr )
        ret ~= p;
    return ret;
}

class SimpleClassifier : Classifier
{
    static class Class
    {
        Skeleton mean;
        Skeleton[] array;

        float diff( in Skeleton s )
        {
            auto mj = mean.allJoints();
            auto sj = s.allJoints();

            float k = 0.0f;

            foreach( i, ref joint; mj )
                k += (joint.pos - sj[i].pos).len;

            return k;
        }

        void append( in Skeleton s )
        {
            auto n = array.length;
            mean = skeleton_div( skeleton_add( skeleton_mlt( mean, n ), s ), n+1 ); 
            array ~= s;
        }
    }

    Skeleton[][] opCall( in Skeleton[][] skel_arr )
    {
        auto red = plainArray( skel_arr );
        Class[] classes;
        foreach( skel; red )
        {
            auto res = findClass( classes, skel );
            Class cur = processResult( classes, res[0], res[1] );
            cur.append( skel );
        }
        return getSkeletons( classes );
    }

    static auto findClass( Class[] classes, in Skeleton skel )
    {
        Class fnd;
        float min_diff = float.max;
        foreach( cls; classes )
        {
            auto df = cls.diff(skel);
            if( df < min_diff )
            {
                min_diff = df;
                fnd = cls;
            }
        }
        return tuple( fnd, min_diff );
    }

    static Class processResult( ref Class[] classes, Class cls, float delta )
    {
        float cls_epsilon = 4.0f;
        auto ret = cls;
        if( cls is null || delta > cls_epsilon )
        {
            ret = new Class;
            classes ~= ret;
        }
        return ret;
    }

    static auto getSkeletons( Class[] classes )
    {
        Skeleton[][] ret;
        foreach( cls; classes )
            ret ~= cls.array;
        return ret;
    }
}

unittest
{
    auto tsc = new SimpleClassifier;

    Skeleton[][] by_tracker;
    by_tracker ~= getFakeSkeletons(vec3(0,0,.1));
    by_tracker ~= getFakeSkeletons(vec3(0,0.1,0));

    auto by_group = tsc( by_tracker );

    /+
    printSkeletonsDArray( by_tracker );
    printSkeletonsDArray( by_group );
    +/

    assert( by_group[0][0] == by_tracker[0][0] );
    assert( by_group[0][1] == by_tracker[1][0] );
    assert( by_group[1][0] == by_tracker[0][1] );
    assert( by_group[1][1] == by_tracker[1][1] );
}

class SimpleComplexer : Complexer
{
    Skeleton[] opCall( in Skeleton[][] skels )
    {
        Skeleton[] result;
        foreach( group; skels )
        {
            if( group.length == 0 ) continue;
            Skeleton buf = group[0];
            if( group.length > 1 )
            {
                size_t n = 1;
                foreach( s; group[1 .. $] )
                    buf = skeleton_div( skeleton_add( skeleton_mlt( buf, n ), s ), ++n ); 
            }
            result ~= buf;
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

class SimpleUserHandler: UserHandler
{
protected:
    bool is_overdue = true;
    User self_user;
    float max_dist2;
public:

    this( User fuser, float max_dist )
    {
        self_user = fuser;
        is_overdue = false;
        max_dist2 = max_dist ^^ 2;
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
        auto dist2 = (self_user.skel.torso.pos - s.torso.pos).len2;
        if( dist2 > max_dist2 ) return 0.0f;
        return 1.0f / ( dist2 + 0.0001f );
    }
}

unittest
{
    auto us0 = getFakeSkeletons(vec3(0,0,0),[vec3(0,0,0)])[0];
    auto tsuh = new SimpleUserHandler( User(0,us0), 1.0f );
    assert( tsuh.updated );
    assert( tsuh.user == User(0,us0) );
    assert( !tsuh.isOverdue );

    assert( tsuh.calcTransformPossibility(us0) >= 0.9f / 0.0001f );
    assert( tsuh.calcTransformPossibility(skeleton_offset(us0,vec3(1.1,0,0))) == 0.0f );
    auto tctpn = tsuh.calcTransformPossibility(skeleton_offset(us0,vec3(0.5,0,0)));
    assert( tctpn > 0.0f );
    assert( tctpn < 1.0f / 0.0001f );
}

class SimpleDestributor : Destributor
{
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
        uhlist ~= new SimpleUserHandler( User(i,s), 1.0f );
    auto skels1 = getFakeSkeletons(vec3(0,0.2,0));
    assert( [] == tsd(uhlist,skels1) );
    import std.array;
    auto uhskels = array( map!(a=>a.user.skel)(uhlist) );
    assert( uhskels == skels1 );
}

class SimpleMultiTrackerFactory : MultiTrackerFactory
{
protected:
    Classifier _classifier;
    Complexer _complexer;
    Destributor _destributor;

    float max_user_transform_dist = 1.0f;

public:
    this()
    {
        _classifier = new SimpleClassifier;
        _complexer = new SimpleComplexer;
        _destributor = new SimpleDestributor;
    }

    @property
    {
        Classifier classifier() { return _classifier; }
        Complexer complexer() { return _complexer; }
        Destributor destributor() { return _destributor; }
    }

    UserHandler newUserHandler( User user )
    { return new SimpleUserHandler( user, max_user_transform_dist ); }
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

