using System;
using System.Collections.Generic;
using System.Collections;
using System.Drawing;
using System.IO;
using System.Windows.Media.Imaging;

class Filter
{

    int filterSize = 5;
    int threshold = 60;
    int nFrames = 10;

    Dictionary<string, string> depthfiles;
    Queue<UInt16[]> averageQueue;


    public Filter()
    {
        averageQueue = new Queue<ushort[]>();
        depthfiles = new Dictionary<string, string>();
    }
    UInt16 filterPixelMedian(UInt16[] depthArray, int depthIndex, int x, int y)
    {
        int offsetplus = (int)Math.Floor(filterSize / 2.0);
        int offsetless = (int)Math.Ceiling(filterSize / 2.0) - 1;
        UInt16[] values = new UInt16[filterSize * filterSize];
        int k = 0;
        for (int i = 0; i < filterSize; i++)
        {
            for (int j = 0; j < filterSize; j++)
            {
                int newx = (x - offsetless + i);
                int newy = (y - offsetless + j);
                //extending borders for convolution
                newx = newx < 0 ? 0 : newx;
                newx = newx >= 512 ? 512 - 1 : newx;
                newy = newy < 0 ? 0 : newy;
                newy = newy >= 424 ? 424 - 1 : newy;
                int idx = (newy * 512) + newx;
                values[k++] = depthArray[idx];
            }
        }
        int halfindex = (int)Math.Floor(filterSize * filterSize / 2.0);
        //orderby values.
        //get the one in 
        Array.Sort(values);
        return values[halfindex];
    }

    UInt16[] medianPixels(UInt16[] depths)
    {
        UInt16[] MedianDepthArray = new UInt16[depths.Length];
        UInt16[] values = new UInt16[averageQueue.Count];

        for (int depthArrayRowIndex = 0; depthArrayRowIndex < 424; depthArrayRowIndex++)
        {
            // Process each pixel in the row
            for (int depthArrayColumnIndex = 0; depthArrayColumnIndex < 512; depthArrayColumnIndex++)
            {
                var index = depthArrayColumnIndex + (depthArrayRowIndex * 512);
                int i = 0;
                foreach (var item in averageQueue)
                {
                    if (Math.Abs(depths[index] - item[index]) < threshold)
                    {
                        values[i] = item[index];
                    }
                    else
                    {
                        values[i] = depths[index];
                    }
                    i++;
                }
                Array.Sort(values);
                int medianIndex = (int)Math.Floor(values.Length / 2.0);
                MedianDepthArray[index] = values[medianIndex];
            }
        }
        return MedianDepthArray;
    }

    void createFileList(string[] files)
    {
        foreach (string s in files)
        {
            if (s.Contains("filtered") || s.Contains("seg")) continue;
            string name = s;
            char[] sep = { '\\' };
            string[] tokens = name.Split(sep);

            name = tokens[tokens.Length - 1];
            name = System.Text.RegularExpressions.Regex.Match(name, @"\d+\W\d+").Value;
            depthfiles.Add(name, s);
        }
    }

    private UInt16[] ReadDepthPixels(string path)
    {
        using (FileStream fs = new FileStream(path, FileMode.Open, FileAccess.Read))
        {
            using (BinaryReader bw = new BinaryReader(fs))
            {
                int nvalues;
                // Write the number of items
                //nvalues = bw.ReadInt32();
                nvalues = 512 * 424;
                UInt16[] res = new UInt16[nvalues];
                int idx = 0;
                for (int i = 0; i < nvalues; i++)
                {
                    res[i] = bw.ReadUInt16();
                    if (res[i] != 0)
                        idx++;
                }
                bw.Dispose();
                fs.Close();
                return res;
            }
        }
    }

    public void writeDepths(UInt16[] depths, string path)
    {
        // Open file for reading
        System.IO.FileStream _FileStream = new System.IO.FileStream(path, System.IO.FileMode.Create, System.IO.FileAccess.Write);
        // Writes a block of bytes to this stream using data from
        BinaryWriter bw = new BinaryWriter(_FileStream);
        // a byte array.
        foreach (UInt16 value in depths)
        {
            bw.Write(value);
        }
        // close file stream
        _FileStream.Close();
    }

    private void CheckForDequeue()
    {
        if (averageQueue.Count > nFrames)
        {
            averageQueue.Dequeue();
            CheckForDequeue();
        }
    }

    public void applyFilters()
    {
        int i = 0;
        int seconds = 0;
        int frame = 0;


        string lastKey = "";
        int missingFrames = 0;
        int iniCount = depthfiles.Count;
        while (i < iniCount)
        {
            string sd = "";
            if (frame == 30)
            {
                frame = 0;
                seconds++;
            }
            string key = seconds + "," + frame;

            if (depthfiles.ContainsKey(key))
            {
                missingFrames = 0;
                lastKey = key;
                i++;
            }
            else
            {
                missingFrames++;
                if (lastKey == "")
                {
                    frame++;
                    continue;
                }

            }
            frame++;
            sd = depthfiles[lastKey];

            UInt16[] depths = ReadDepthPixels(sd);

            int k = 0;

            UInt16[] filteredDepths = new UInt16[depths.Length];
            for (int m = 0; m < 424; m++)
            {
                for (int n = 0; n < 512; n++)
                {

                    filteredDepths[k] = filterPixelMedian(depths, k, n, m);
                    k++;
                }
            }
            depths = filteredDepths;



            averageQueue.Enqueue(depths);
            CheckForDequeue();
            depths = medianPixels(depths);

            string[] tokens = sd.Split('\\');
            tokens[tokens.Length - 2] = "depthbgFiltered";
            string np = tokens[0];
            for(int j = 1; j < tokens.Length; j++)
                np = np + "\\" + tokens[j];

            if (!sd.Contains("filtered")) {
                writeDepths(depths, np);
            }
          
            depthfiles[key] = np;


        }
    }


    static void Main(string[] args)
    {
        Filter f = new Filter();
        f.createFileList(Directory.GetFiles(args[0]));
        f.applyFilters();
        return;
    }

}


