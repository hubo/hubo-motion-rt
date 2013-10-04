/********************************************************/
/*    Author: Matt Zucker <mzucker1@swarthmore.edu>     */
/* Copyright (c) 2013 Matt Zucker. All rights reserved. */
/********************************************************/

#ifndef _SIMPLECONFIG_H_
#define _SIMPLECONFIG_H_

#include <string>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <map>
#include <exception>
#include <fstream>
#include "strutils.h"


class SimpleConfig {
public:

  typedef std::map<std::string, std::pair<std::string, bool> > StringMap;

  StringMap lookup;

  SimpleConfig() {}

  SimpleConfig(const std::string& filename) {
    parse(filename);
  }

  void parseCommaSeparated(const std::string& data, bool requireExisting) {
    size_t start = 0;
    while ( start < data.length() ) {

      size_t cpos = data.find(',', start);
      size_t end;
      size_t nextstart;

      if (cpos == std::string::npos) {
	end = data.length();
	nextstart = cpos;
      } else {
	end = cpos;
	nextstart = cpos+1;
      }

      size_t len = end-start;

      std::string assignment = data.substr(start, len);
      std::string s1, s2;

      if (!split(assignment, '=', s1, s2)) {
	std::cerr << "error parsing comma separated assignment " << assignment << "\n";
	exit(1);
      }
      
      StringMap::iterator i = lookup.find(s1);
      if (i == lookup.end()) { 
	if (requireExisting) {
	  std::cerr << "setting unknown key '" << s1 << "' from comma separated assignment\n";
	  exit(1);
	} else {
	  lookup[s1] = std::make_pair(s2, false);
	}
      } else {
	i->second.first = s2;
      }


      std::cout << "got segment '" << data.substr(start, len) << "'\n";

      start = nextstart;


    }
  }

  void parse(const std::string& filename) { 

    std::ifstream istr(filename.c_str());

    if (!istr.is_open()) { 
      std::cerr << "error opening config " << filename << "\n";
      exit(1);
    }

    std::string line;

    while (std::getline(istr,line)) {
      size_t p = line.find('#');
      if (p != std::string::npos) { 
        line = line.substr(0, p);
      }
      std::string s1, s2;
      if (line.length() > 9 && line.substr(0, 8) == "include ") {
        std::string baseDir = directoryOf(filename);
        std::string filename = trimws(line.substr(8, line.length()-8));
        parse(combineDir(baseDir, filename));
      } else if (!split(line, '=', s1, s2)) {
        if (!trimws(line).empty()) {
          std::cerr << "error parsing line " << line << "\n";
          exit(1);
        }
      } else {
        //std::cout << "set '" << s1 << "' to '" << s2 << "'\n";
        lookup[s1] = std::make_pair(s2, false);
      }
    }

  }

  const std::string& get(const std::string& key) {
    StringMap::iterator i = lookup.find(key);
    if (i == lookup.end()) { 
      std::cerr << "config key not found: " << key << "\n";
      exit(1);
    }
    i->second.second = true;
    return i->second.first;
  }

  template <class Tval>
  void get(const std::string& key, Tval& val) {

    const std::string& sval = get(key);

    std::istringstream istr(sval);

    if ( !(istr >> val) || (istr.peek() != EOF)) {
      std::cerr << "error parsing value for " << key << "\n";
      exit(1);
    }

  }

  bool getBool(const std::string& key) { 
    const std::string& sval = lower(get(key));
    if (sval == "true" || sval == "1") { 
      return true;
    } else if (sval == "false" || sval == "0") {
      return false;
    } else {
      std::cerr << "error parsing boolean value for " << key << "\n";
      exit(1);
    }
  }

  template <class Tenum>
  Tenum getEnum(const std::string& key, 
                const std::map<std::string, Tenum>& values) {

    const std::string& val = get(key);
    typename std::map<std::string, Tenum>::const_iterator i = values.find(val);
    if (i == values.end()) { 
      std::cerr << "invalid value for " << key << ": " << val << "\n";
      exit(1);
    }

    return i->second;

  }

  void checkUsed(bool abortIfUnused) const {
    bool unused = false;
    for (StringMap::const_iterator i = lookup.begin(); i!=lookup.end(); ++i) {
      if (!i->second.second) { 
        std::cerr << "*** CONFIG ITEM " << i->first << " WAS SET BUT NEVER READ! ***\n";
        unused = true;
      }
    }
    if (unused && abortIfUnused) {
      exit(1);
    }
  }

};

#endif
