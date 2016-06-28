#define DATA_TYPE uchar

__kernel void OverlapingEdges(int cols,int rows,int src_step,int dst_step,int channels,
                       int pixelBuff, __global const DATA_TYPE* src, __global DATA_TYPE* dst)
{
    const int x = get_global_id(0);
    const int y = get_global_id(1);
    int dst_idx = y * dst_step + x;
    
    int color = 0;
    for(int xRange = -pixelBuff; xRange < pixelBuff; xRange++){
    	for(int yRange = -pixelBuff; yRange < pixelBuff; yRange++){
    		if((y + yRange)>=0 && (y + yRange)<rows && (x + xRange) >= 0 && (x + xRange) < cols){
    			int src_idx = (y + yRange) * src_step + (x + xRange) * channels;
    			color |= src[src_idx];
    		}
    	}
    }
    if(color < 20){
    	dst[dst_idx] = 0;
    }
}