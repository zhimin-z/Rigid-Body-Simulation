#ifndef WORLDLOADER_H
#define WORLDLOADER_H

#include <string>
#include "World.h"

World LoadWorldFromFile(std::string const& filename);

#endif