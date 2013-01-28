/* -*- mode: C; c-basic-offset: 2  -*- */
/*
 * Copyright (c) 2008, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * Shared Memory Layout:
 *
 *    ________
 *   | Header |
 *   |--------|
 *   | GUARDH |
 *   |--------|
 *   | Index  |
 *   |        |
 *   |        |
 *   |--------|
 *   | GUARDI |
 *   |--------|
 *   |  Data  |
 *   |        |
 *   |        |
 *   |        |
 *   |        |
 *   |        |
 *   |        |
 *   |--------|
 *   | GUARDD |
 *   |________|
 */

#ifndef ACH_H
#define ACH_H

#ifdef __cplusplus

#define ACH_RESTRICT
extern "C" {
#else

#define ACH_RESTRICT restrict
#endif

#define ACH_CHAN_NAME_MAX 64

#define ACH_INTR_RETRY 8

#define ACH_SHM_MAGIC_NUM 0xb07511f3


#define ACH_SHM_GUARD_HEADER_NUM ((uint64_t)0x1A2A3A4A5A6A7A8A)

#define ACH_SHM_GUARD_INDEX_NUM ((uint64_t)0x1B2B3B4B5B6B7B8B)

#define ACH_SHM_GUARD_DATA_NUM ((uint64_t)0x1C2C3C4C5C6C7C8C)

  typedef enum {
    ACH_OK = 0,         
    ACH_OVERFLOW = 1,       
    ACH_INVALID_NAME = 2,   
    ACH_BAD_SHM_FILE = 3,   
    ACH_FAILED_SYSCALL = 4, 
    ACH_STALE_FRAMES = 5,   
    ACH_MISSED_FRAME = 6,   
    ACH_TIMEOUT = 7,        
    ACH_EEXIST = 8,          
    ACH_ENOENT = 9,        
    ACH_CLOSED = 10
  } ach_status_t;

  typedef enum {
    ACH_MODE_PUBLISH,  
    ACH_MODE_SUBSCRIBE  
  } ach_mode_t;

  typedef enum {
    ACH_CHAN_STATE_INIT,    
    ACH_CHAN_STATE_RUN,     
    //ACH_CHAN_STATE_READING, ///< readers using channel
    //ACH_CHAN_STATE_WRITING, ///< writer modifying channel
    ACH_CHAN_STATE_CLOSED 
  } ach_chan_state_t;

  typedef struct {
    uint32_t magic;          
    size_t len;              
    size_t index_cnt;        
    size_t data_size;        
    size_t data_head;        
    size_t data_free;        
    size_t index_head;       
    size_t index_free;       
    int state;  
    int refcount; 
    int anon; 
    char name[1+ACH_CHAN_NAME_MAX]; 
    struct /* anonymous structure */ {
      pthread_mutex_t mutex;     
      pthread_cond_t cond; 
      int dirty;
    } sync; 
    // should force our alignment to 8-bytes...
    uint64_t last_seq;       
  } ach_header_t;

  typedef struct {
    size_t size;      
    size_t offset;    
    uint64_t seq_num; 
  } ach_index_t ;


  typedef struct {
    int map_anon; 
    ach_header_t *shm;   
  } ach_attr_t;


  
  typedef struct {
    int truncate; 
    int map_anon; 
    ach_header_t *shm; 
  } ach_create_attr_t;


  typedef struct {
    ach_header_t *shm; 
    size_t len;        
    int fd;            
    uint64_t seq_num;  
    size_t next_index; 
    ach_attr_t attr; 
    int mode; 
  } ach_channel_t;


#define ACH_SHM_GUARD_HEADER( shm ) ((uint64_t*)((ach_header_t*)(shm) + 1))

#define ACH_SHM_INDEX( shm ) ((ach_index_t*)(ACH_SHM_GUARD_HEADER(shm) + 1))

#define ACH_SHM_GUARD_INDEX( shm ) \
  ((uint64_t*)(ACH_SHM_INDEX(shm) + ((ach_header_t*)(shm))->index_cnt))

#define ACH_SHM_DATA( shm ) ( (uint8_t*)(ACH_SHM_GUARD_INDEX(shm) + 1) )

#define ACH_SHM_GUARD_DATA( shm ) \
  ((uint64_t*)(ACH_SHM_DATA(shm) + ((ach_header_t*)(shm))->data_size))


  void ach_attr_init( ach_attr_t *attr );

  void ach_create_attr_init( ach_create_attr_t *attr );

  int ach_create( char *channel_name, 
                  size_t frame_cnt, size_t frame_size,
                  ach_create_attr_t *attr);

  int ach_open(ach_channel_t *chan, const char *channel_name,
               ach_attr_t *attr);


  int ach_get_next(ach_channel_t *chan, void *buf, size_t size, size_t *frame_size);

  int ach_wait_next(ach_channel_t *chan, void *buf, size_t size, size_t *frame_size, 
                    const struct timespec *ACH_RESTRICT abstime);

  int ach_get_last(ach_channel_t *chan, void *buf, 
                   size_t size, size_t *frame_size);

  int ach_copy_last(ach_channel_t *chan, void *buf, 
                    size_t size, size_t *frame_size);
    

  int ach_wait_last( ach_channel_t *chan, void *buf, size_t size, size_t *frame_size, 
                     const struct timespec *ACH_RESTRICT abstime);

  int ach_put(ach_channel_t *chan, void *buf, size_t len);


  int ach_flush( ach_channel_t *chan );

  int ach_close(ach_channel_t *chan);


  const char *ach_result_to_string(ach_status_t result);


  void ach_dump( ach_header_t *shm);

#define ACH_STREAM_PREFIX_SIZE  12

  int ach_stream_write_msg( int fd, const char *buf, size_t cnt);

  int ach_stream_read_msg_size( int fd, int *cnt);

  int ach_stream_read_msg_data( int fd, char *buf, size_t msg_size, size_t buf_size);


  int ach_stream_read_fill( int fd, char *buf, size_t cnt );

  int ach_stream_write_fill( int fd, const char *buf, size_t cnt );

  int ach_read_line( int fd, char *buf, size_t cnt );

  int ach_chmod( ach_channel_t *chan, mode_t mode );

#ifdef __cplusplus
}
#endif

#endif // ACH_H