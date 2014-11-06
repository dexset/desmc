module des.mc.calibrate.multipoint;

import des.mc.calibrate.point;
import des.mc.calibrate.util;

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

    auto bmpc = new BaseMultiPointCalibrator( 2, PointCalibratorParam(10,0.15,0.4,0.9) );

    bool normalCalcFirstPoint = false;
    foreach( i; 0 .. 20 )
    {
        auto q = bmpc.filter( rays[cur_ray] + rndSeg() );
        if( q.state == PointCalibrationResult.State.ACCEPTED )
        {
            assert( abs( i - 10 ) < 2 );
            normalCalcFirstPoint = true;
            break;
        }
    }
    assert(normalCalcFirstPoint);

    void appendNextRay()
    {
        cur_ray++;
        foreach( i; 0 .. 20 )
        {
            auto yx = bmpc.filter( rays[cur_ray] + rndSeg() );
            if( yx.state == PointCalibrationResult.State.ACCEPTED ) break;
        }
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

interface MultiPointCalibratorPrinter
{
    void nextPoint();
    void accepted(size_t,PointCalibrationResult);
    void aborted(size_t,PointCalibrationResult);
    void done(in vec3[]);
}

class CBMultiPointCalibrator : BaseMultiPointCalibrator
{
    MultiPointCalibratorPrinter printer;

    this( size_t point_count, in PointCalibratorParam pcp,
            MultiPointCalibratorPrinter prntr=null ) 
    {
        super( point_count, pcp );
        printer = prntr;
    }

    override PointCalibrationResult filter( in fSeg seg )
    {
        auto res = super.filter( seg );
        if( printer )
        {
            if( res.state == PointCalibrationResult.State.ACCEPTED )
                printer.accepted( currentIndex, res );
            if( res.state == PointCalibrationResult.State.ABORTED ) 
                printer.aborted( currentIndex, res );
            if( done ) printer.done( points );
        }
        return res;
    }

    override void nextPoint()
    {
        super.nextPoint();
        if( printer ) printer.nextPoint();
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

    size_t next_point_call_count = 0;

    class TestPrinter : MultiPointCalibratorPrinter
    {
        void nextPoint() { next_point_call_count++; }
        void accepted(size_t,PointCalibrationResult) { cur_ray++; }
        void aborted(size_t,PointCalibrationResult) {}
        void done(in vec3[]) {}
    }

    auto buf_len = 10;
    auto cbmpc = new CBMultiPointCalibrator( 2, PointCalibratorParam(buf_len,0.15,0.4), new TestPrinter );
    auto eachmp = new EachMultiPointCalibrator( cbmpc );

    size_t steps = 0;
    while( !eachmp.done )
    {
        steps++;
        eachmp.filter( rays[cur_ray]+rndSeg() );
    }
    assert( abs( steps - buf_len * 4 ) < 3 );
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
        return acos(dot( a, b )) > min_next_point_angle;
    }
}
