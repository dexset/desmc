module desmc.core.skeleton;

import std.stdio;
import std.string;
import std.algorithm;
public import desmath.linear;

struct Joint
{
    vec3 pos;
    float qual = 0.0f;
}

struct Skeleton
{
    enum JointID
    {
        HEAD,
        NECK,

        LEFT_SHOULDER,
        RIGHT_SHOULDER,
        LEFT_ELBOW,
        RIGHT_ELBOW,
        LEFT_HAND,
        RIGHT_HAND,

        TORSO,

        LEFT_HIP,
        RIGHT_HIP,
        LEFT_KNEE,
        RIGHT_KNEE,
        LEFT_FOOT,
        RIGHT_FOOT
    }

    enum JointCount = JointID.max-JointID.min+1;

    private Joint[JointCount] joints;

    mixin( accessArray!("joints",Joint,JointID) );

    auto transform( in mat4 mtr ) const
    {
        Skeleton ret;
        foreach( i, j; joints )
            ret.joints[i] = Joint( (mtr * vec4(j.pos,1.0)).xyz, j.qual );
        return ret;
    }

    vec3 center() const
    {
        vec3 p;
        foreach( joint; joints )
            p += joint.pos;
        return p / joints.length;
    }

    Joint[JointCount] allJoints() const { return joints; }
    void setJoints( in Joint[JointCount] j ) { joints = j; }

    vec3[] allBones() const
    {
        vec3[] ret;
        ret ~= joints[JointID.HEAD].pos;
        ret ~= joints[JointID.NECK].pos;

        ret ~= joints[JointID.NECK].pos;
        ret ~= joints[JointID.LEFT_SHOULDER].pos;

        ret ~= joints[JointID.NECK].pos;
        ret ~= joints[JointID.RIGHT_SHOULDER].pos;

        ret ~= joints[JointID.RIGHT_ELBOW].pos;
        ret ~= joints[JointID.RIGHT_SHOULDER].pos;

        ret ~= joints[JointID.LEFT_ELBOW].pos;
        ret ~= joints[JointID.LEFT_SHOULDER].pos;

        ret ~= joints[JointID.LEFT_ELBOW].pos;
        ret ~= joints[JointID.LEFT_HAND].pos;

        ret ~= joints[JointID.RIGHT_ELBOW].pos;
        ret ~= joints[JointID.RIGHT_HAND].pos;

        ret ~= joints[JointID.TORSO].pos;
        ret ~= joints[JointID.NECK].pos;

        ret ~= joints[JointID.TORSO].pos;
        ret ~= joints[JointID.LEFT_HIP].pos;

        ret ~= joints[JointID.TORSO].pos;
        ret ~= joints[JointID.RIGHT_HIP].pos;

        ret ~= joints[JointID.RIGHT_KNEE].pos;
        ret ~= joints[JointID.RIGHT_HIP].pos;

        ret ~= joints[JointID.LEFT_KNEE].pos;
        ret ~= joints[JointID.LEFT_HIP].pos;

        ret ~= joints[JointID.LEFT_KNEE].pos;
        ret ~= joints[JointID.LEFT_FOOT].pos;

        ret ~= joints[JointID.RIGHT_KNEE].pos;
        ret ~= joints[JointID.RIGHT_FOOT].pos;
        return ret;
    }

    static Skeleton fromJoints( in Joint[JointCount] jj )
    {
        Skeleton ret;
        ret.setJoints( jj );
        return ret;
    }
}

version(unittest)
{
    Joint[Skeleton.JointCount] pose_norm;
    Skeleton utest_skeleton;
    static this()
    {
        pose_norm[Skeleton.JointID.HEAD] = Joint( vec3(0,0,2), 1.0f );
        pose_norm[Skeleton.JointID.NECK] = Joint( vec3(0,0,1.8), 1.0f );

        pose_norm[Skeleton.JointID.LEFT_SHOULDER]  = Joint( vec3(0, 0.2,1.7), 1.0f );
        pose_norm[Skeleton.JointID.RIGHT_SHOULDER] = Joint( vec3(0,-0.2,1.7), 1.0f );
        pose_norm[Skeleton.JointID.LEFT_ELBOW]     = Joint( vec3(0, 0.3,1.2), 1.0f );
        pose_norm[Skeleton.JointID.RIGHT_ELBOW]    = Joint( vec3(0,-0.3,1.2), 1.0f );
        pose_norm[Skeleton.JointID.LEFT_HAND]      = Joint( vec3(0.2,0.3,0.9), 1.0f );
        pose_norm[Skeleton.JointID.RIGHT_HAND]     = Joint( vec3(0.2,-0.3,0.9), 1.0f );

        pose_norm[Skeleton.JointID.TORSO] = Joint( vec3(0,0,1.2), 1.0f );

        pose_norm[Skeleton.JointID.LEFT_HIP] = Joint( vec3(0,0.15,1), 1.0f );
        pose_norm[Skeleton.JointID.RIGHT_HIP] = Joint( vec3(0,-0.15,1), 1.0f );
        pose_norm[Skeleton.JointID.LEFT_KNEE] = Joint( vec3(0,0.15,0.5), 1.0f );
        pose_norm[Skeleton.JointID.RIGHT_KNEE] = Joint( vec3(0,-0.15,0.5), 1.0f );
        pose_norm[Skeleton.JointID.LEFT_FOOT] = Joint( vec3(0,0.15,0), 1.0f );
        pose_norm[Skeleton.JointID.RIGHT_FOOT] = Joint( vec3(0,-0.15,0), 1.0f );
        utest_skeleton = Skeleton.fromJoints( pose_norm );
    }

    Skeleton[] getFakeSkeletons( vec3 global_offset = vec3(0,0,0),
            vec3[] offsets = [ vec3(0,2,0), vec3(1,-1,0) ] )
    {
        Skeleton[] ret;
        foreach( i, offset; offsets )
            ret ~= skeleton_offset( utest_skeleton, global_offset + offset );
        return ret;
    }
}

unittest
{
    auto ts = Skeleton.fromJoints( pose_norm );
    assert( pose_norm[Skeleton.JointID.HEAD] == ts.head );
    assert( pose_norm[Skeleton.JointID.RIGHT_HAND] == ts.rightHand );
}

Skeleton skeleton_mlt( in Skeleton s, float v )
{
    auto s_joints = s.allJoints();
    foreach( ref joint; s_joints )
    {
        joint.pos *= v;
        joint.qual *= v;
    }
    return Skeleton.fromJoints( s_joints );
}

unittest
{
    auto ts = Skeleton.fromJoints( pose_norm );
    auto mts = skeleton_mlt( ts, 10 );
    assert( pose_norm[Skeleton.JointID.HEAD].pos * 10 == mts.head.pos );
}

Skeleton skeleton_add( in Skeleton a, in Skeleton b )
{
    auto a_joints = a.allJoints();
    auto b_joints = b.allJoints();
    foreach( i, ref joint; a_joints )
    {
        joint.pos += b_joints[i].pos;
        joint.qual += b_joints[i].qual;
    }
    return Skeleton.fromJoints( a_joints );
}

unittest
{
    auto ts1 = Skeleton.fromJoints( pose_norm );
    auto ts2 = Skeleton.fromJoints( pose_norm );
    auto rts = skeleton_add( ts1, ts2 );
    assert( pose_norm[Skeleton.JointID.HEAD].pos * 2 == rts.head.pos );
}

Skeleton skeleton_diff( in Skeleton a, in Skeleton b )
{
    auto a_joints = a.allJoints();
    auto b_joints = b.allJoints();
    foreach( i, ref joint; a_joints )
    {
        joint.pos -= b_joints[i].pos;
        joint.qual -= b_joints[i].qual;
    }
    return Skeleton.fromJoints( a_joints );
}

unittest
{
    auto ts1 = Skeleton.fromJoints( pose_norm );
    auto ts2 = skeleton_mlt( ts1, 2 );
    auto rts = skeleton_diff( ts2, ts1 );
    assert( pose_norm[Skeleton.JointID.HEAD].pos == rts.head.pos );
}

Skeleton skeleton_div( in Skeleton s, float v )
{ return skeleton_mlt( s, 1.0f / v ); }

Skeleton skeleton_offset( in Skeleton a, in vec3 offset )
{
    auto a_joints = a.allJoints();
    foreach( ref joint; a_joints )
        joint.pos += offset;
    return Skeleton.fromJoints( a_joints );
}

private
{
    @property string accessArray(string arrayName,arrayType,Enum)()
    {
        string[] ret;

        string type = arrayType.stringof;

        foreach( e; Enum.min .. Enum.max )
        {
            string id = format("%s.%s",Enum.stringof,e);
            string name = toProperName( id );
            string access = format( "%s[%s]", arrayName, id );
            ret ~= format( "@property ref const(%1$s) %2$s() const { return %3$s; }\n" ~
                        "@property void %2$s( in %1$s v ) { %3$s = v; }",
                        type, name, access );
        }

        return ret.join("\n");
    }

    string toProperName( string str )
    {
        auto words = str.split(".")[$-1].split("_");
        auto res = toLowerPure( words[0] );
        if( words.length > 1 )
            res = reduce!( (a,b) => a ~ b )( res, map!( a => a.capitalize )(words[1 .. $]) );
        return res;
    }

    unittest
    {
        assert( toProperName( "RIGHT_HAND" ) == "rightHand" );
        assert( toProperName( "RIGHT_HAND_FIRST_FINGER" ) == "rightHandFirstFinger" );
        assert( toProperName( "HEAD" ) == "head" );
    }

    pure string toLowerPure( string str )
    {
        auto diff = cast(ubyte)('A') - cast(ubyte)('a');
        char[] buf;
        auto upperStart = cast(ubyte)('A');
        auto upperEnd = cast(ubyte)('Z');
        foreach( c; str )
        {
            auto bc = cast(ubyte)(c);
            if( bc >= upperStart && bc <= upperEnd )
                buf ~= cast(char)(bc-diff);
            else
                buf ~= c;
        }

        return buf.idup;
    }

    unittest
    {
        assert( toLowerPure( "OlolOlo" ) == "olololo" );
        assert( toLowerPure( "RIGHT" ) == "right" );
        assert( toLowerPure( "r10%Z" ) == "r10%z" );
        string rndstr = "`1234567890-=qwertyuiop[]\\asdfghjkl;'zxcvbnm,./~!@#$%^&*()_+QWERTYUIOP{}|ASDFGHJKL:\"ZXCVBNM<>?";
        assert( toLowerPure( rndstr ) == rndstr.toLower );
    }
}
