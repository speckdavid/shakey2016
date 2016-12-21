#!/usr/bin/env python
# Script that adds an object to the database
# Modified from object_recognition_core

from __future__ import print_function 
import os, sys, argparse
from object_recognition_core.db import models
from object_recognition_core.db.tools import args_to_db_params
import object_recognition_core.db.tools as dbtools
import couchdb

def parse_args():
    parser = argparse.ArgumentParser(description='Add an object to the db.',
                                      fromfile_prefix_chars='@')
    parser.add_argument('-n', '--object_name', metavar='OBJECT_NAME', dest='object_name', type=str, default='')
    parser.add_argument('-d', '--description', metavar='DESCRIPTION', dest='description', type=str, default='')
    parser.add_argument('-a', '--author', metavar='AUTHOR_NAME', dest='author_name', type=str, default='')
    parser.add_argument('-e', '--email', metavar='EMAIL_ADDRESS', dest='author_email', type=str, default='')
    parser.add_argument('tags', metavar='TAGS', type=str, nargs='*', help='Tags to add to object description.')
    dbtools.add_db_arguments(parser)

    args = parser.parse_args()

    return args

if __name__ == '__main__':
    args = parse_args()
    obj = models.Object(object_name=args.object_name,
                        description=args.description,
                        tags=args.tags,
                        author_name=args.author_name,
                        author_email=args.author_email,
                        )

    couch = couchdb.Server(args.db_root)
    db = dbtools.init_object_databases(couch)
    objects = db
    if args.commit:
        obj.store(objects)
        print(obj.id)
    else:
        print('Use the --commit option to commit the proposed change.')
        sys.exit(1)
