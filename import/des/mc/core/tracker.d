module des.mc.core.tracker;

import des.mc.core.skeleton;
import des.mc.core.user;

interface Tracker
{
    User[] getUsers();
}

version(unittest)
{
    final class FakeTracker : Tracker
    {
        private User[] users;
        this( in vec3 offset=vec3(0,0,0) )
        { users = getFakeUsers( offset, [ vec3(0,2,0), vec3(1,-1,0) ] ); }
        User[] getUsers() { return users; }
    }
}

class TransformTracker : Tracker
{
protected:
    abstract @property User[] users();
    abstract Skeleton transform( in Skeleton );

public:
    final User[] getUsers()
    {
        auto src = users;
        User[] dst;
        dst.length = src.length;

        foreach( i, user; src )
            dst[i] = User( user.id, transform( user.skel ) );

        return dst;
    }
}

class MatrixTransformTracker : TransformTracker
{
protected:
    override @property User[] users()
    { return source.getUsers(); }

    override Skeleton transform( in Skeleton skel )
    { return skel.transform( mtr ); }

    Tracker source;
    mat4 mtr;

public:
    this( Tracker src, in mat4 tr )
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

unittest
{
    auto ltr = new LookAtTransform;
    ltr.pos = vec3( 0, -1, 0 );
    ltr.target = vec3( 0, -1, -1 );
    ltr.up = vec3( 0, 1, 0 );

    auto ft = new FakeTracker;
    auto mtt = new MatrixTransformTracker( ft, ltr.matrix );
    auto uu = ft.getUsers();
    auto ut = mtt.getUsers();
    assert( ut[0].skel.head.pos == (ltr.matrix * vec4(uu[0].skel.head.pos,1.0) ).xyz );
}
