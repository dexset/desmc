import std.stdio;
import desmc;

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
