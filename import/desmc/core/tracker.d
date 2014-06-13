module desmc.core.tracker;

import desmc.core.skeleton;
import desmc.core.user;

interface Tracker
{
    User[] getUsers();
}

interface TransformTracker : Tracker
{
protected:
    @property User[] users();
    Skeleton transform( in Skeleton );

public:
    final User[] getUsers()
    {
        auto src = users;
        User[] dst;
        dst = src.length;

        foreach( i, user; src )
            dst[i] = User( user.id, transform( skel ) );

        return dst;
    }
}

class MatrixTransformTracker : TransformTracker
{
protected:
    @property User[] users()
    { return source.getUsers(); }

    Skeleton transform( in Skeleton skel )
    { return skel.transform( mtr ); }

    Tracker source;
    mat4 mtr;

public:
    this( Tracker src, in mtr4 tr )
    {
        source = src;
        mtr = tr;
    }

    @property
    {
        Tracker sourceTracker() { return source; }
        void sourceTracker( Tracker src ) { source = src; }

        mat4 transformMatrix() const { return mtr; }
        void transformMatrix( in mat4 tr ) { mtr = tr; }
    }
}
