module desmc.multitrack.simplemodel;

import desmc.multitrack.model;
import std.algorithm;

class SimpleClassifier : Classifier
{
    static class Class
    {
        Skeleton mean;
        Skeleton[] array;

        this( Skeleton s ) { append( s ); }

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
        Skeleton[] red;
        reduce!( (a,b) => a ~= b )( red, skel_arr );
        Class[] classes;
        foreach( skel; red )
        {
            Class cur;
            float min_diff = float.max;
            foreach( cls; classes )
            {
                auto df = cls.diff(skel);
                if( df < min_diff )
                {
                    min_diff = df;
                    cur = cls;
                }
            }
            if( cur is null )
            {
                cur = new Class(skel);
                classes ~= cur;
            }
            else
            {
                cur.append( skel );
            }
        }
        Skeleton[][] ret;
        foreach( cls; classes )
            ret ~= cls.array;
        return ret;
    }
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
                    buf = skeleton_div( skeleton_add( skeleton_mlt( buf, n ), s ), n++ ); 
            }
            result ~= buf;
        }
        return result;
    }
}

class SimpleUserHandler: UserHandler
{
protected:
    bool is_overdue = true;
    User self_user;
public:

    this( User fuser )
    {
        self_user = fuser;
        is_overdue = false;
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
        auto dist2 = (self_user.skel.center - s.center).len2;
        if( dist2 > 1 ) return 0.0f;
        return 1.0f / ( dist2 + 0.0001 );
    }
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

class SimpleMultiTrackerFactory : MultiTrackerFactory
{
protected:
    Classifier _classifier;
    Complexer _complexer;
    Destributor _destributor;

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
    { return new SimpleUserHandler( user ); }
}
