
2021-09-14

Two issues I can find

1) There appera to be two diff versions of the S/W that were installed over time
confusing which directory and config file were the correct ne to use.
"CounterManager.txt"  and  "LarcosCounterManager.txt".   The exe in use 
was using "LarcosCounterManager.txt" while the sourve code on the system
was making use of "CounterManager.txt".

2) There are particles who reach the very edge of the camera line such that
the code to extract a particle tried to extrat one more column beyond tje 
edge(0-2047); When it tries to access 2048 thre was a arrays bound check



CReated new access token:
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

