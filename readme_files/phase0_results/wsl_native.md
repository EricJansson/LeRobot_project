# Phase 0 offline benchmark

Status: **OFFLINE VERIFIED**

UTC run time: 2026-11-05T15:46:43.281831+00:00

No motors, serial connections, calibration writes or torque operations.

Experimental model/exclusions only; physical and continuous clearance unverified.

| Case/stage | Median ms | p95 ms | p99 ms | Worst ms | >20 ms | Accepted/total |
|---|---:|---:|---:|---:|---:|---:|
| neutral/ik | 0.050 | 0.093 | 0.227 | 0.905 | 0 | 500/500 |
| neutral/collision | 0.046 | 0.106 | 0.184 | 0.402 | 0 | 500/500 |
| neutral/combined/0.5deg/0mm | 0.184 | 0.553 | 0.697 | 1.219 | 0 | 500/500 |
| neutral/combined/0.5deg/2mm | 0.169 | 0.330 | 0.461 | 1.591 | 0 | 500/500 |
| neutral/combined/0.5deg/5mm | 0.166 | 0.306 | 0.424 | 0.731 | 0 | 500/500 |
| neutral/combined/1deg/0mm | 0.169 | 0.366 | 0.594 | 1.042 | 0 | 500/500 |
| neutral/combined/1deg/2mm | 0.162 | 0.233 | 0.378 | 0.524 | 0 | 500/500 |
| neutral/combined/1deg/5mm | 0.169 | 0.304 | 0.531 | 0.730 | 0 | 500/500 |
| neutral/combined/2deg/0mm | 0.164 | 0.339 | 0.501 | 0.870 | 0 | 500/500 |
| neutral/combined/2deg/2mm | 0.170 | 0.326 | 0.506 | 0.626 | 0 | 500/500 |
| neutral/combined/2deg/5mm | 0.166 | 0.295 | 0.442 | 0.709 | 0 | 500/500 |
| extended/ik | 0.050 | 0.078 | 0.116 | 0.486 | 0 | 500/500 |
| extended/collision | 0.034 | 0.046 | 0.074 | 0.145 | 0 | 500/500 |
| extended/combined/0.5deg/0mm | 0.201 | 0.340 | 0.491 | 0.587 | 0 | 500/500 |
| extended/combined/0.5deg/2mm | 0.209 | 0.365 | 0.482 | 0.574 | 0 | 500/500 |
| extended/combined/0.5deg/5mm | 0.205 | 0.340 | 0.526 | 0.768 | 0 | 500/500 |
| extended/combined/1deg/0mm | 0.162 | 0.268 | 0.490 | 0.580 | 0 | 500/500 |
| extended/combined/1deg/2mm | 0.164 | 0.236 | 0.380 | 0.774 | 0 | 500/500 |
| extended/combined/1deg/5mm | 0.164 | 0.252 | 0.441 | 0.661 | 0 | 500/500 |
| extended/combined/2deg/0mm | 0.162 | 0.244 | 0.461 | 0.578 | 0 | 500/500 |
| extended/combined/2deg/2mm | 0.163 | 0.297 | 0.465 | 0.576 | 0 | 500/500 |
| extended/combined/2deg/5mm | 0.165 | 0.240 | 0.435 | 0.573 | 0 | 500/500 |
| near_singularity/ik | 0.050 | 0.092 | 0.255 | 0.319 | 0 | 500/500 |
| near_singularity/collision | 0.033 | 0.045 | 0.084 | 0.134 | 0 | 500/500 |
| near_singularity/combined/0.5deg/0mm | 0.205 | 0.338 | 0.509 | 0.761 | 0 | 500/500 |
| near_singularity/combined/0.5deg/2mm | 0.205 | 0.348 | 0.510 | 0.666 | 0 | 500/500 |
| near_singularity/combined/0.5deg/5mm | 0.205 | 0.357 | 0.546 | 0.767 | 0 | 500/500 |
| near_singularity/combined/1deg/0mm | 0.164 | 0.301 | 0.414 | 0.545 | 0 | 500/500 |
| near_singularity/combined/1deg/2mm | 0.164 | 0.238 | 0.456 | 0.640 | 0 | 500/500 |
| near_singularity/combined/1deg/5mm | 0.164 | 0.246 | 0.423 | 0.708 | 0 | 500/500 |
| near_singularity/combined/2deg/0mm | 0.162 | 0.248 | 0.457 | 0.617 | 0 | 500/500 |
| near_singularity/combined/2deg/2mm | 0.162 | 0.271 | 0.420 | 0.689 | 0 | 500/500 |
| near_singularity/combined/2deg/5mm | 0.165 | 0.267 | 0.439 | 0.663 | 0 | 500/500 |
| near_limit/ik | 0.050 | 0.097 | 0.221 | 0.322 | 0 | 500/500 |
| near_limit/collision | 0.034 | 0.054 | 0.147 | 0.308 | 0 | 500/500 |
| near_limit/combined/0.5deg/0mm | 0.162 | 0.248 | 0.467 | 0.674 | 0 | 500/500 |
| near_limit/combined/0.5deg/2mm | 0.167 | 0.274 | 0.449 | 0.618 | 0 | 500/500 |
| near_limit/combined/0.5deg/5mm | 0.167 | 0.300 | 0.524 | 0.806 | 0 | 500/500 |
| near_limit/combined/1deg/0mm | 0.166 | 0.338 | 0.469 | 0.562 | 0 | 500/500 |
| near_limit/combined/1deg/2mm | 0.163 | 0.283 | 0.473 | 0.705 | 0 | 500/500 |
| near_limit/combined/1deg/5mm | 0.168 | 0.322 | 0.471 | 0.681 | 0 | 500/500 |
| near_limit/combined/2deg/0mm | 0.165 | 0.332 | 0.530 | 1.226 | 0 | 500/500 |
| near_limit/combined/2deg/2mm | 0.166 | 0.259 | 0.503 | 0.577 | 0 | 500/500 |
| near_limit/combined/2deg/5mm | 0.166 | 0.294 | 0.438 | 0.725 | 0 | 500/500 |
| wrist_roll/ik | 0.056 | 0.088 | 0.148 | 0.633 | 0 | 500/500 |
| wrist_roll/collision | 0.033 | 0.046 | 0.080 | 0.332 | 0 | 500/500 |
| wrist_roll/combined/0.5deg/0mm | 0.209 | 0.365 | 0.531 | 0.720 | 0 | 500/500 |
| wrist_roll/combined/0.5deg/2mm | 0.208 | 0.311 | 0.549 | 0.756 | 0 | 500/500 |
| wrist_roll/combined/0.5deg/5mm | 0.210 | 0.357 | 0.533 | 0.655 | 0 | 500/500 |
| wrist_roll/combined/1deg/0mm | 0.169 | 0.308 | 0.522 | 0.718 | 0 | 500/500 |
| wrist_roll/combined/1deg/2mm | 0.170 | 0.324 | 0.529 | 0.830 | 0 | 500/500 |
| wrist_roll/combined/1deg/5mm | 0.171 | 0.319 | 0.448 | 0.708 | 0 | 500/500 |
| wrist_roll/combined/2deg/0mm | 0.172 | 0.347 | 0.469 | 0.542 | 0 | 500/500 |
| wrist_roll/combined/2deg/2mm | 0.168 | 0.251 | 0.511 | 0.758 | 0 | 500/500 |
| wrist_roll/combined/2deg/5mm | 0.171 | 0.348 | 0.503 | 0.683 | 0 | 500/500 |
| unreachable/ik | 0.691 | 1.114 | 1.500 | 1.550 | 0 | 0/500 |
| unreachable/collision | 0.033 | 0.047 | 0.072 | 0.163 | 0 | 500/500 |
| unreachable/combined/0.5deg/0mm | 0.685 | 1.109 | 1.344 | 2.600 | 0 | 0/500 |
| unreachable/combined/0.5deg/2mm | 0.678 | 1.036 | 1.390 | 1.533 | 0 | 0/500 |
| unreachable/combined/0.5deg/5mm | 0.679 | 1.125 | 1.418 | 1.540 | 0 | 0/500 |
| unreachable/combined/1deg/0mm | 0.687 | 1.161 | 1.399 | 2.031 | 0 | 0/500 |
| unreachable/combined/1deg/2mm | 0.671 | 1.164 | 1.424 | 1.940 | 0 | 0/500 |
| unreachable/combined/1deg/5mm | 0.682 | 1.138 | 1.456 | 2.184 | 0 | 0/500 |
| unreachable/combined/2deg/0mm | 0.676 | 1.167 | 1.579 | 1.942 | 0 | 0/500 |
| unreachable/combined/2deg/2mm | 0.675 | 1.117 | 1.380 | 1.611 | 0 | 0/500 |
| unreachable/combined/2deg/5mm | 0.676 | 1.066 | 1.335 | 1.774 | 0 | 0/500 |
| collision_near/ik | 0.049 | 0.083 | 0.174 | 0.385 | 0 | 500/500 |
| collision_near/collision | 0.051 | 0.095 | 0.203 | 0.245 | 0 | 500/500 |
| collision_near/combined/0.5deg/0mm | 0.206 | 0.373 | 0.528 | 0.765 | 0 | 500/500 |
| collision_near/combined/0.5deg/2mm | 0.841 | 1.404 | 1.689 | 3.912 | 0 | 500/500 |
| collision_near/combined/0.5deg/5mm | 1.312 | 1.680 | 1.886 | 2.248 | 0 | 0/500 |
| collision_near/combined/1deg/0mm | 0.209 | 0.347 | 0.452 | 0.707 | 0 | 500/500 |
| collision_near/combined/1deg/2mm | 0.827 | 1.193 | 1.389 | 1.654 | 0 | 500/500 |
| collision_near/combined/1deg/5mm | 1.312 | 1.713 | 1.945 | 2.096 | 0 | 0/500 |
| collision_near/combined/2deg/0mm | 0.207 | 0.330 | 0.538 | 0.701 | 0 | 500/500 |
| collision_near/combined/2deg/2mm | 0.842 | 1.215 | 1.392 | 1.516 | 0 | 500/500 |
| collision_near/combined/2deg/5mm | 1.309 | 1.705 | 1.865 | 1.998 | 0 | 0/500 |
| retraction/ik | 0.080 | 0.135 | 0.287 | 0.405 | 0 | 500/500 |
| retraction/collision | 0.033 | 0.054 | 0.121 | 0.168 | 0 | 500/500 |
| retraction/combined/0.5deg/0mm | 1.134 | 1.600 | 1.765 | 2.438 | 0 | 500/500 |
| retraction/combined/0.5deg/2mm | 1.133 | 1.601 | 1.918 | 2.394 | 0 | 500/500 |
| retraction/combined/0.5deg/5mm | 1.142 | 1.617 | 1.815 | 2.123 | 0 | 500/500 |
| retraction/combined/1deg/0mm | 0.641 | 0.968 | 1.148 | 1.333 | 0 | 500/500 |
| retraction/combined/1deg/2mm | 0.647 | 0.999 | 1.248 | 1.562 | 0 | 500/500 |
| retraction/combined/1deg/5mm | 0.657 | 1.002 | 1.284 | 1.684 | 0 | 500/500 |
| retraction/combined/2deg/0mm | 0.426 | 0.710 | 0.935 | 1.095 | 0 | 500/500 |
| retraction/combined/2deg/2mm | 0.428 | 0.638 | 0.848 | 1.095 | 0 | 500/500 |
| retraction/combined/2deg/5mm | 0.433 | 0.694 | 0.900 | 1.154 | 0 | 500/500 |
| interior_collision/path/0.5deg/0mm | 6.824 | 7.344 | 7.654 | 8.314 | 0 | 0/500 |
| interior_collision/path/0.5deg/2mm | 9.029 | 9.924 | 10.469 | 12.020 | 0 | 0/500 |
| interior_collision/path/0.5deg/5mm | 10.412 | 11.631 | 12.729 | 14.012 | 0 | 0/500 |
| interior_collision/path/1deg/0mm | 3.750 | 5.308 | 5.945 | 6.952 | 0 | 0/500 |
| interior_collision/path/1deg/2mm | 4.722 | 6.399 | 7.303 | 8.255 | 0 | 0/500 |
| interior_collision/path/1deg/5mm | 5.206 | 5.862 | 6.069 | 6.967 | 0 | 0/500 |
| interior_collision/path/2deg/0mm | 1.784 | 2.405 | 2.540 | 2.708 | 0 | 0/500 |
| interior_collision/path/2deg/2mm | 2.560 | 3.210 | 3.500 | 4.593 | 0 | 0/500 |
| interior_collision/path/2deg/5mm | 2.531 | 3.158 | 3.372 | 4.032 | 0 | 0/500 |

See the accompanying JSON for raw samples, outcomes, model hashes, exclusions and dependencies.
