# hifive-test

This is a test program for figuring out the ESP SOLO chip on the hifive1-revb board.

If you don't have one it won't be interesting to you.

Clone with --recursive, it has a hacked version of the riscv-rust hifive1 repo. Stdout now does some input, too. Sorry.

So far, things work, but info and error messages don't come through. Example:

```
Command: 'AT+CWJAP="T35.24_guest","welcome guest!"'
> AT+CWJAP="T35.24_guest","welcome guest!"

< AT+CWJAP="T35.24_guest","welcome guest!"

< WIFI DISCONNECT

< WIFI CONNECTED

< WIFI GOT IP

<
OK

Command: 'AT+CIPSTART="TCP","tauben.newby.org",80'
> AT+CIPSTART="TCP","tauben.newby.org",80

< CONNECT

<
OK

Command: 'AT+PING="192.168.1.204"'
> AT+PING="192.168.1.204"

< +PING:TIMEOUT

<
ERROR
```

Informational things do not work:

```
Command: 'AT+CIPSTATUS'
> AT+CIPSTATUS

```
(crickets).
