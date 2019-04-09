using System;
using HeliSharp;
using ManyConsole.CommandLineUtils;
using System.Collections.Generic;

namespace CsHeli
{
    class Program
    {
        public static int Main(string[] args)
        {
            var commands = GetCommands();
            return ConsoleCommandDispatcher.DispatchCommand(commands, args, Console.Out);
        }

        public static IEnumerable<ConsoleCommand> GetCommands()
        {
            return ConsoleCommandDispatcher.FindCommandsInSameAssemblyAs(typeof(Program));
        }

    }
}
