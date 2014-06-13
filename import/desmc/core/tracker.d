module desmc.core.tracker;

import desmc.core.skeleton;
import desmc.core.user;

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
        {
            auto np = Skeleton.fromJoints( pose_norm );
            users ~= User( 0, skeleton_offset( np, vec3(0,2,0) + offset ) );
            users ~= User( 0, skeleton_offset( np, vec3(1,-1,0) + offset ) );
        }
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
    import desmath.linear.camera;
    auto mtr = _lookAt( vec3(0,-1,0), vec3(0,-1,-1), vec3(0,1,0) );
    auto ft = new FakeTracker;
    auto mtt = new MatrixTransformTracker( ft, mtr );
    auto uu = ft.getUsers();
    auto ut = mtt.getUsers();
    assert( ut[0].skel.head.pos == (mtr * vec4(uu[0].skel.head.pos,1.0) ).xyz );
}
