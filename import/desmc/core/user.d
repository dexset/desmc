module desmc.core.user;

public import desmc.core.skeleton;

struct User
{
    size_t id;
    Skeleton skel;
}

version(unittest)
{
    User[] getFakeUsers( vec3 global_offset = vec3(0,0,0),
            vec3[] offsets = [ vec3(0,2,0), vec3(1,-1,0) ] )
    {
        auto skels = getFakeSkeletons( global_offset, offsets );
        User[] ret;
        foreach( i, skel; skels )
            ret ~= User( i, skel );
        return ret;
    }
}

