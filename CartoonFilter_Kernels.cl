#define DATA_TYPE uchar

__kernel void OverlapingEdges(int cols,int rows,int src_step,int dst_step,int channels,
                       int pixelBuff, __global const DATA_TYPE* src, __global DATA_TYPE* dst)
{
    const int x = get_global_id(0);
    const int y = get_global_id(1);
    int dst_idx = y * dst_step + x;
    
    int color = 0;
    for(int xStart = -pixelBuff; xStart <= 0; xStart++){
    	int yStart = -pixelBuff - xStart; 
    	for(int i = 0; i <= pixelBuff; i++){
    		int xRange = (x + xStart + i), yRange = (y + yStart + i);
    		if(yRange >= 0 && yRange < rows && xRange >= 0 && xRange < cols){
    			int src_idx = yRange * src_step + xRange * channels;
    			color |= src[src_idx];
    			
	    		yRange -= 1;
	    		if(xStart && i && yRange >= 0 && yRange < rows){
	    			int src_idx = yRange * src_step + xRange * channels;
	    			color |= src[src_idx];
	    		}
    		}
    	}
    }
    
    if(color < 20){
    	dst[dst_idx] = 0;
    }
}