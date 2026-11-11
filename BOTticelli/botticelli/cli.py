"""Consistent argparse help for commands and subcommands."""
import argparse


def command(commands, name, summary, example):
    return commands.add_parser(name, help=summary, description=summary,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        epilog='Example: ' + example)
