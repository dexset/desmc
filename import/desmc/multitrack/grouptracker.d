module desmc.multitrack.multitracker;

import desmc.multitrack.model;
import desmc.core;

import std.algorithm;

class MultiTracker : Tracker
{
protected:

    Tracker[] trackers;
    UserHandler[] handlers;

    Classifier classifier;
    Complexer complexer;
    Destributor destributor;
    GroupTrackerFactory factory;

public:
    this( Tracker[] trs, MultiTrackerFactory gtf )
    {
        trackers = trs;
        setFactory( gtf );
    }

    void setFactory( MultiTrackerFactory gtf )
    {
        factory = gtf;
        classifier = factory.classifier;
        complexer = factory.complexer;
        destributor = factory.destributor;
    }

    User[] getUsers()
    {
        auto skels = getReadySkeletons();
        destributeSkeletonsToHandlers( skels );
        return map!( a => a.user )( filter!( a => a.updated )(handlers) );
    }

protected:

    Skeleton[] getReadySkeletons()
    {
        auto by_trackers = getSkeletonsByTrackers();
        auto by_users = classifier( by_trackers );
        return complexer( by_users );
    }

    Skeleton[][] getSkeletonsByTrackers()
    {
        Skeleton[][] ret;
        foreach( tracker; trackers )
            ret ~= map!(a=>a.skel)(tracker.getUsers());
        return ret;
    }

    void destributeSkeletonsToHandlers( Skeleton[] skels )
    {
        setHandlersOverdue();
        auto not_destributed = destributor( handlers, skels );
        addNewHandlers( not_destributed );
        cleanUpOverdueHandlers();
    }

    void setHandlersOverdue() { foreach( uh; handlers ) uh.setOverdue(); }

    void addNewHandlers( Skeleton[] skels )
    {
        foreach( skel; skels )
            handlers ~= factory.newUserHandler( User(freeUserId, skel) );
    }

    @property size_t freeUserId() { return users_id++; }
    size_t users_id = 0;

    void cleanUpOverdueHandlers()
    {
        UserHandler[] good;
        foreach( uh; handlers )
            if( !uh.isOverdue )
                good ~= uh;
        handlers = good;
    }
}
