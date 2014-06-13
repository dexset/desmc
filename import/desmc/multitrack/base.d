module desmc.multitrack.base;

import desmc.multitrack.model;

class BaseClassifier : Classifier
{
    Skeleton[][] opCall( in Skeleton[][] )
    {
        //TODO
    }
}

class BaseComplexer : Complexer
{
    Skeleton[] opCall( in Skeleton[][] )
    {
        // TODO

    }
}

class BaseUserHandler: UserHandler
{
protected:
    size_t overdue_count = 0;
    size_t max_overdue;

    User self_user;
public:

    this( User fuser, size_t maxOD=4 )
    {
        self_user = fuser;
        max_overdue = maxOD;
    }

    @property 
    {
        bool updated() const { return !overdue_count; }
        ref const(User) user() const { return self_user; }
        bool isOverdue() const { return overdue_count >= max_overdue; }
    }

    void setOverdue() { overdue_count++; }

    void setSkeleton( in Skeleton s )
    {
        self_user.skel = s;
        overdue_count = 0;
    }

    float calcTransformPossibility( in Skeleton s ) const
    {
        // TODO
    }
}

class BaseDestributor : Destributor
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
                handlers[max_j].setSkeleton( skels[max_i] );
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

class BaseMultiTrackerFactory : MultiTrackerFactory
{
protected:
    BaseClassifier baseClassifier;
    BaseComplexer baseComplexer;
    BaseDestributor baseDestributor;

    size_t max_handler_overdue = 4;

public:
    this()
    {
        baseClassifier = new BaseClassifier;
        baseComplexer = new BaseComplexer;
        baseDestributor = new BaseDestributor;
    }

    @property
    {
        Classifier classifier() { return baseClassifier; }
        Complexer complexer() { return baseComplexer; }
        Destributor destributor() { return baseDestributor; }
    }

    UserHandler newUserHandler( User user )
    {
        return new BaseUserHandler( user, max_handler_overdue );
    }
}
