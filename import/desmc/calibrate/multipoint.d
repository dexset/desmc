module desmc.calibrate.multipoint;

import desmc.calibrate.point;
import desmc.calibrate.util;

float toRad( float deg ) pure nothrow { return deg / 180.0 * PI; }

interface MultiPointCalibrator
{
    void reset();
    void nextPoint();
    PointCalibrationResult filter( in fSeg seg );

    const @property
    {
        const(vec3)[] points();
        bool done();
        size_t currentIndex();
    }
}

class BaseMultiPointCalibrator : MultiPointCalibrator
{
protected:
    PointCalibrator[] calibrators;
    size_t current;
    PointCalibratorParam calibrator_params;

public:
    this( size_t point_count, PointCalibratorParam pcp )
    {
        calibrator_params = pcp;
        foreach( i; 0 .. point_count )
            calibrators ~= new PointCalibrator( pcp );
    }

    void reset()
    {
        foreach( pc; calibrators )
            pc = new PointCalibrator( calibrator_params );
    }

    void nextPoint() { current = ( current + 1 ) % calibrators.length; }

    PointCalibrationResult filter( in fSeg seg )
    { return calibrators[current].filter( seg ); }

    const @property
    {
        const(vec3)[] points() { return array( map!(a=>a.point)( calibrators ) ); }
        bool done() { return all(points); }
        size_t currentIndex() { return current; }
    }
}

unittest
{
    fSeg[] rays = 
        [
        fSeg( vec3(0,0,-0.1), vec3(1,1,0) ), fSeg( vec3(2,0,0.1), vec3(-1,1,0) ),
        fSeg( vec3(0,3,0), vec3(2,1,0) ), fSeg( vec3(0,4,0), vec3(1,0,0) ),
        ];

    size_t cur_ray = 0;

    vec3[] res = [ vec3(1,1,0), vec3(2,4,0) ];

    auto bmpc = new BaseMultiPointCalibrator( 2, PointCalibratorParam(10,0.15,0.5,0.9) );

    bool normalCalcFirstPoint = false;
    foreach( i; 0 .. 20 )
    {
        auto q = bmpc.filter( rays[cur_ray] + rndSeg() );
        if( q.state == PointCalibrationResult.State.ACCEPTED )
        {
            assert( i > 8 && i < 12 );
            normalCalcFirstPoint = true;
            break;
        }
    }
    assert(normalCalcFirstPoint);

    void appendNextRay()
    {
        cur_ray++;
        while( bmpc.filter( rays[cur_ray] + rndSeg() ).state != 
                PointCalibrationResult.State.ACCEPTED ) {}
    }

    appendNextRay();

    assert( !bmpc.done );
    assert( bmpc.currentIndex == 0 );
    bmpc.nextPoint();
    assert( bmpc.currentIndex == 1 );
    appendNextRay();
    assert( !bmpc.done );
    appendNextRay();
    assert( bmpc.done );
    import std.range;
    assert( all!(a=>a.len2<0.1)( map!(a=>a[0]-a[1])( zip(bmpc.points, res) ) ) );
}

class CBMultiPointCalibrator : BaseMultiPointCalibrator
{
    this( size_t point_count, in PointCalibratorParam pcp ) { super( point_count, pcp ); }

    alias void delegate() EmptyCallback;
    alias void delegate(size_t, PointCalibrationResult) FilterCallback;
    alias void delegate( in vec3[] ) ResultCallback;

    EmptyCallback next_point_callback;
    FilterCallback accepted_callback;
    FilterCallback aborted_callback;
    ResultCallback done_callback;

    override PointCalibrationResult filter( in fSeg seg )
    {
        auto res = super.filter( seg );
        if( res.state == PointCalibrationResult.State.ACCEPTED &&
                accepted_callback ) accepted_callback( currentIndex, res );
        if( res.state == PointCalibrationResult.State.ABORTED &&
                aborted_callback ) aborted_callback( currentIndex, res );
        if( done && done_callback ) done_callback( points );
        return res;
    }

    override void nextPoint()
    {
        super.nextPoint();
        if( next_point_callback ) next_point_callback();
    }
}

abstract class BehaviorMultiPointCalibrator : MultiPointCalibrator
{
protected:
    MultiPointCalibrator calibrator;

public:
    this( MultiPointCalibrator mpc ) { calibrator = mpc; }

    void reset() { calibrator.reset(); }
    void nextPoint() { calibrator.nextPoint(); }

    const @property
    {
        const(vec3)[] points() { return calibrator.points; }
        bool done() { return calibrator.done; }
        size_t currentIndex() { return calibrator.currentIndex; }
    }
}

/+ калибровка каждой точки по отдельности +/
class EachMultiPointCalibrator : BehaviorMultiPointCalibrator
{
    this( MultiPointCalibrator mpc ) { super(mpc); }

    auto filter( in fSeg seg )
    {
        auto res = calibrator.filter( seg );
        if( res.done ) nextPoint();
        return res;
    }
}

unittest
{
    fSeg[] rays = 
        [
        fSeg( vec3(0,0,-0.1), vec3(1,1,0) ), fSeg( vec3(2,0,0.1), vec3(-1,1,0) ),
        fSeg( vec3(0,3,0), vec3(2,1,0) ), fSeg( vec3(0,4,0), vec3(1,0,0) ),
        ];

    size_t cur_ray = 0;

    vec3[] res = [ vec3(1,1,0), vec3(2,4,0) ];

    auto cbmpc = new CBMultiPointCalibrator( 2, PointCalibratorParam(10,0.15) );
    size_t next_point_call_count = 0;
    cbmpc.next_point_callback = ()
    { 
        next_point_call_count++; 
    };
    cbmpc.accepted_callback = (size_t i, PointCalibrationResult pcr) { cur_ray++; };
    auto eachmp = new EachMultiPointCalibrator( cbmpc );

    size_t steps = 0;
    while( !eachmp.done )
    {
        steps++;
        eachmp.filter( rays[cur_ray]+rndSeg() );
    }
    assert( steps > 39 && steps < 45 );
    assert( next_point_call_count == 2 );
    import std.range;
    assert( all!(a=>a.len2<0.1)( map!(a=>a[0]-a[1])( zip(eachmp.points, res) ) ) );
}

/+ калибровка серии точек из одного положения +/
class SeriesMultiPointCalibrator : BehaviorMultiPointCalibrator
{
protected:
    float min_next_point_angle;

public:
    this( MultiPointCalibrator mpc, float mnpa )
    {
        super(mpc);
        min_next_point_angle = abs(mnpa);
    }

    override auto filter( in fSeg seg )
    {
        auto res = calibrator.filter( seg );
        if( res.state != PointCalibrationResult.State.BUFFERED && 
                checkRay( res.avg_ray ) ) nextPoint();
        return res;
    }

protected:

    fSeg last_ray;

    bool checkRay( in fSeg ray )
    {
        if( last_ray.dir.len == 0 )
        {
            last_ray = ray;
            return true;
        }

        auto a = last_ray.dir.e;
        auto b = ray.dir.e;
        return acos(a^b) > min_next_point_angle;
    }
}
