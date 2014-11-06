import std.stdio;
import des.mc;

void main()
{
    version(unittest)
    {
        writeln( "\n------------------------------" ); 
        writeln( "DES MoCap unittesting complite" ); 
        writeln( "------------------------------\n" ); 
    }
    else
    {
        stderr.writeln( "build with -unittest flag to test DES MoCap" );
    }
}
