import os, sys, inspect

from numpy import unicode

VERSION = "2.0.18"
REVISION = "ff4ae9f5cc05353d3dc3aeff8854ae69"
GIT = "x"
INSTALL = ''
C1 = '#('
C2 = '#&'
C3 = '#%'
cwd = os.getcwd()
join = os.path.join

WAF = 'waf'


def b(x):
    return x


if sys.hexversion > 0x300000f:
    WAF = 'waf3'


    def b(x):
        return x.encode()


def err(m):
    print(('\033[91mError: %s\033[0m' % m))
    sys.exit(1)


def unpack_wafdir(dir, src):
    f = open(src, 'rb')
    c = 'corrupt archive (%d)'
    while 1:
        line = f.readline()
        if not line:
            err('run waf-light from a folder containing waflib')
        if line == b('#==>\n'):
            txt = f.readline()
            if not txt: err(c % 1)
            if f.readline() != b('#<==\n'): err(c % 2)
            break
    if not txt: err(c % 3)
    txt = txt[1:-1].replace(b(C1), b('\n')).replace(b(C2), b('\r')).replace(b(C3), b('\x00'))

    import shutil, tarfile
    try:
        shutil.rmtree(dir)
    except OSError:
        pass
    try:
        for x in ('Tools', 'extras'):
            os.makedirs(join(dir, 'waflib', x))
    except OSError:
        err("Cannot unpack waf lib into %s\nMove waf in a writable directory" % dir)

    os.chdir(dir)
    tmp = 't.bz2'
    t = open(tmp, 'wb')
    try:
        t.write(txt)
    finally:
        t.close()

    try:
        t = tarfile.open(tmp)
    except:
        try:
            os.system('bunzip2 t.bz2')
            t = tarfile.open('t')
            tmp = 't'
        except:
            os.chdir(cwd)
            try:
                shutil.rmtree(dir)
            except OSError:
                pass
            err("Waf cannot be unpacked, check that bzip2 support is present")

    try:
        for x in t: t.extract(x)
    finally:
        t.close()

    for x in ('Tools', 'extras'):
        os.chmod(join('waflib', x), 493)

    if sys.hexversion < 0x300000f:
        sys.path = [join(dir, 'waflib')] + sys.path
        import fixpy2
        fixpy2.fixdir(dir)

    os.remove(tmp)
    os.chdir(cwd)

    try:
        dir = unicode(dir, 'mbcs')
    except:
        pass
    try:
        from ctypes import windll
        windll.kernel32.SetFileAttributesW(dir, 2)
    except:
        pass


def test(dir):
    try:
        os.stat(join(dir, 'waflib'))
        return os.path.abspath(dir)
    except OSError:
        pass


def find_lib():
    src = os.path.abspath(inspect.getfile(inspect.getmodule(err)))
    base, name = os.path.split(src)

    # devs use $WAFDIR
    w = test(os.environ.get('WAFDIR', ''))
    if w: return w

    # waf-light
    if name.endswith('waf-light'):
        w = test(base)
        if w: return w
        for dir in sys.path:
            if test(dir):
                return dir
        err('waf-light requires waflib -> export WAFDIR=/folder')

    dirname = '%s-%s-%s' % (WAF, VERSION, REVISION)
    for i in (INSTALL, '/usr', '/usr/local', '/opt'):
        w = test(i + '/lib/' + dirname)
        if w: return w

    # waf-local
    dir = join(base, (sys.platform != 'win32' and '.' or '') + dirname)
    w = test(dir)
    if w: return w

    # unpack
    unpack_wafdir(dir, src)
    return dir


wafdir = find_lib()
sys.path.insert(0, wafdir)

if __name__ == '__main__':



    from waflib import Scripting

    Scripting.waf_entry_point(cwd, VERSION, wafdir)  # https://waf.io/apidocs/_modules/waflib/Scripting.html
    print(wafdir)
