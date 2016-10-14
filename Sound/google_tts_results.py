#!/usr/bin/python
#-*- mode: python; coding: utf-8 -*-
# import urllib, urllib2
from urllib.request import urlopen
import urllib
from os import path

def get_tts_mp3( lang, sent, fname=None ):
    print ("Retrieving .mp3 for sentence: %s" % (sent))
    baseurl  = "http://translate.google.com/translate_tts"
    values   = { 'q': sent, 'tl': lang }
    data     = urllib.parse.urlencode(values)
    request  = urllib.request.Request(baseurl, str.encode(data))
    request.add_header("User-Agent", "Mozilla/5.0 (X11; U; Linux i686) Gecko/20071127 Firefox/2.0.0.11" )
    response = urllib.request.urlopen(request)
    if( fname==None ):
        fname = "_".join(sent.split())
    ofp = open(fname,"wb")
    ofp.write(response.read())
    print ("Saved to file: %s" % fname)
    return 

def parse_list_from_file( lang, infile ):
    ifp = open(infile)
    for line in ifp:
        line = line.strip()
        get_tts_mp3( lang, line, fname=line+".mp3" )
    ifp.close()
    return

if __name__=="__main__":
    import sys, argparse

    example = "%s --lang ja --sent 日本語は難しい --fname outfile.mp3" % sys.argv[0]

    parser  = argparse.ArgumentParser( description=example )
    parser.add_argument('--lang',   "-a", help='Input language abbreviation: Korean=kr, Japanese=ja, English=en, etc.', required=True )
    parser.add_argument('--sent',   "-s", help='An input sentence to synthesize.', default=None )
    parser.add_argument('--slist',  "-l", help='A file with a list of sentences to synthesize, one per line.', default=None )
    parser.add_argument('--fname',  "-f", help='Output filename, defaults to the input sentence .mp3', default=None )
    args = parser.parse_args()
   
    if not args.sent==None:
        get_tts_mp3( args.lang, args.sent, args.fname )
    elif not args.slist==None:
        parse_list_from_file( args.lang, args.slist )
    else:
        print ("Please supply either a value for either --sent or --slist.")