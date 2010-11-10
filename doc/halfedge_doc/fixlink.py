import glob, string

files = glob.glob('./html/*.html');

old_link = "../../ttl_doc/html/namespacettl_util.html";
new_link = "../../ttl_doc/html/namespacettl__util.html";

for file in files:
    f = open(file, 'r');
    txt = f.read();
    txt = string.replace(txt, old_link, new_link);
    f.close();
    f = open(file, 'w');
    f.write(txt);
    f.close();
