using System.Collections;
using System.Collections.Generic;
using System;

namespace ReedsShepp
{

    public class CourseSegmentType
    {
        string segment1;
        string segment2;
        string segment3;
        public string ctypes;
        public CourseSegmentType(string segment1, string segment2, string segment3)
        {
            this.segment1 = segment1;
            this.segment2 = segment2;
            this.segment3 = segment3;
            ctypes = segment1 + "," + segment2 + "," + segment3;
        }
    }

}
