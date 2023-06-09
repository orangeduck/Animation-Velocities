#pragma once

#include "common.h"
#include "vec.h"
#include "quat.h"
#include "array.h"

#include <assert.h>
#include <float.h>
#include <stdio.h>
#include <math.h>

//--------------------------------------

struct database
{
    array2d<vec3> bone_positions;
    array2d<quat> bone_rotations;
    array1d<int> bone_parents;
    
    array1d<int> range_starts;
    array1d<int> range_stops;
    
    int nframes() const { return bone_positions.rows; }
    int nbones() const { return bone_positions.cols; }
    int nranges() const { return range_starts.size; }
};

void database_load(database& db, const char* filename)
{
    FILE* f = fopen(filename, "rb");
    assert(f != NULL);
    
    array2d_read(db.bone_positions, f);
    array2d_read(db.bone_rotations, f);
    array1d_read(db.bone_parents, f);
    
    array1d_read(db.range_starts, f);
    array1d_read(db.range_stops, f);
    
    fclose(f);
}
