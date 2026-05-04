#!/usr/bin/env python2
"""Diagnostic probe for ALColorBlobDetection and ALMemory keys.
Usage: diag_color_blob.py [ROBOT_IP] [PORT]
If ROBOT_IP omitted, reads ROBOT_IP env var or prompts.
"""
from naoqi import ALProxy
import sys, time, traceback, os

IP = None
PORT = 9559
if len(sys.argv) > 1:
    IP = sys.argv[1]
if len(sys.argv) > 2:
    try:
        PORT = int(sys.argv[2])
    except Exception:
        pass
if IP is None:
    IP = os.environ.get('ROBOT_IP')
if IP is None:
    print('Please provide robot IP as first arg or set ROBOT_IP env var')
    sys.exit(2)

print('Connecting to', IP, PORT)
try:
    mem = ALProxy('ALMemory', IP, PORT)
    cb = ALProxy('ALColorBlobDetection', IP, PORT)
    vid = ALProxy('ALVideoDevice', IP, PORT)
except Exception as e:
    print('Proxy creation failed:', e)
    traceback.print_exc()
    sys.exit(1)

print('ColorBlobDetection methods (filtered):')
for m in dir(cb):
    if any(x in m.lower() for x in ('sub', 'get', 'list', 'color')):
        print(' ', m)

print('\nAttempting setColorSpace(0)')
try:
    cb.setColorSpace(0)
    print(' setColorSpace OK')
except Exception as e:
    print(' setColorSpace failed:', e)

name = 'DiagColorBlob_%d' % int(time.time())
print('\nAttempting subscribe(%s)' % name)
sub_ok = False
try:
    cb.subscribe(name)
    sub_ok = True
    print(' subscribe OK')
except Exception as e:
    print(' subscribe failed:', e)

# Give detector a moment to publish
print('\nWaiting 1.5s for ALMemory keys to appear...')
time.sleep(1.5)

# Try ALMemory helper probes
print('\nProbing ALMemory for Color/Blob keys:')
try:
    fn = getattr(mem, 'getDataListRegisteredInModule', None)
    if fn is None:
        print(' getDataListRegisteredInModule: not available')
    else:
        try:
            keys = fn('ALColorBlobDetection')
            print(' getDataListRegisteredInModule returned', len(keys) if keys else 0)
            for k in (keys or [])[:200]:
                if any(s in k for s in ('Color', 'color', 'Blob', 'blob')):
                    print('  KEY:', k)
        except Exception as e:
            print(' getDataListRegisteredInModule call failed:', e)
except Exception as e:
    print('getDataListRegisteredInModule probe error:', e)

# Fallback to getDataList() if available
try:
    fn2 = getattr(mem, 'getDataList', None)
    if fn2 is None:
        print(' getDataList: not available')
    else:
        try:
            all_keys = fn2()
            filtered = [k for k in all_keys if any(s in k for s in ('Color', 'color', 'Blob', 'blob'))][:200]
            print(' getDataList() found', len(filtered), 'matching keys (showing up to 200)')
            for k in filtered[:200]:
                print('  KEY:', k)
        except Exception as e:
            print(' getDataList() call failed:', e)
except Exception as e:
    print('getDataList probe error:', e)

# Direct probes
probe_keys = [
    'ColorBlobDetection/blobs', 'ColorBlobDetection/Blobs',
    'ColorBlobDetection/LastDetection', 'ColorBlobDetected',
    'ALColorBlobDetection/blobs', 'ALColorBlobDetection/Blobs'
]
print('\nDirect getData probes:')
for k in probe_keys:
    try:
        v = mem.getData(k)
        print(' getData(%s) -> type=%s len=%s' % (k, type(v), (len(v) if v else 0)))
    except Exception as e:
        print(' getData(%s) failed: %s' % (k, e))

# Watch dynamic updates a few times
print('\nWatching ColorBlobDetection/blobs for 8 seconds:')
for i in range(8):
    try:
        v = mem.getData('ColorBlobDetection/blobs')
        print(' loop %d -> %s' % (i, ('None' if not v else ('%d entries' % len(v)))))
    except Exception:
        print(' loop %d -> <no data>' % i)
    time.sleep(1.0)

if sub_ok:
    try:
        cb.unsubscribe(name)
        print('\nunsubscribed', name)
    except Exception:
        pass

print('\nDone')
