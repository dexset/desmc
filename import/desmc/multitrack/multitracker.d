module desmc.multitrack.multitracker;

import desmc.multitrack.model;
import desmc.core;

import std.algorithm;
import std.array;

class MultiTracker : Tracker
{
protected:

    Tracker[] trackers;
    UserHandler[] handlers;

    Heuristic heuristic;
    Classifier classifier;
    Complexer complexer;
    Destributor destributor;
    MultiTrackerFactory factory;

public:
    this( Tracker[] trs, MultiTrackerFactory gtf )
    {
        trackers = trs;
        setFactory( gtf );
    }

    void setTrackers( Tracker[] trs )
    {
        trackers = trs;
    }

    void setFactory( MultiTrackerFactory gtf )
    {
        factory = gtf;
        heuristic = factory.heuristic;
        classifier = factory.classifier;
        complexer = factory.complexer;
        destributor = factory.destributor;
    }

    User[] getUsers()
    {
        auto skels = getReadySkeletons();
        destributeSkeletonsToHandlers( skels );
        return getRespectableUsers();
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
            ret ~= array( map!(a=>heuristic(a.skel))(tracker.getUsers()) );
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

    User[] getRespectableUsers()
    {
        return array( map!(a=>a.user)( filter!(a=>a.respectable)(handlers) ) ).dup;
    }
}
