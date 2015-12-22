/*
 * mm.c - simple light-weight memory allocation package. implementation based on first fit
 * search of a segregated fits table. seg_lists (segregated fits table header) is an array
 * of ptrs to linkedlists of free blocks for each size class. each ptr can be a null ref or a
 * memory address of the next n+1 free block's payload. storing addresses in payload requires a
 * 16 byte block size (including 4 byte header and 4 byte footer). table lookup is based on naive
 * hash to find ideal class size. from this point classes are traversed until first fit is found.
 * this strategy combined with block splitting leads to high throughput and utilization performance
 * of a best bit strategy.
 *
 * e.g.
 *
 * seg_lists
 * -------------------------------------------------------------------
 * |seg_lists[0]:class 1 ---> free block 1 ---> free block n ---> 0
 * -------------------------------------------------------------------
 * |....
 * -------------------------------------------------------------------
 * | seg_lists[SEG_LIST_COUNT]:class n ---> 0
 * -------------------------------------------------------------------
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>

#include "mm.h"
#include "memlib.h"

//CONSTANTS
#define WSIZE 			4
#define DSIZE 			8
#define ALIGNMENT		8
#define CHUNKSIZE		( 1 << 12 )
#define MIN_BLOCK_SIZE  	( 2 * DSIZE )
#define SEG_LIST_COUNT		8

//MACROS
#define MAX( x, y ) 		( ( x ) > ( y ) ? ( x ) : ( y ) )
#define ALIGN( size ) 		( ( ( size ) + ( ALIGNMENT-1 ) ) & ~0x7 )
#define PACK( size, alloc ) 	( ( size ) | ( alloc ) )

#define GET( p )		( *( unsigned int*)( p ) )
#define PUT( p, val ) 		( *(unsigned int*)( p ) = ( val ) )
#define GET_PTR_ADDR( p )	( *( unsigned int**)( p ) )
#define PUT_PTR_ADDR( p, val )	( *( unsigned int**)( p ) = ( val ) )

#define GET_SIZE( p )		( GET( p ) & ~0x7 )
#define GET_ALLOC( p )		( GET( p ) & 0x1 )
#define SIZE_T_SIZE 		( ALIGN( sizeof( size_t ) ) )

#define GET_HEADER( p )		( (char*)( p ) - WSIZE )
#define GET_FOOTER( p )		( (char*)( p ) + GET_SIZE( GET_HEADER( p ) ) - DSIZE )
#define GET_NEXT(p)		( (char*)( p ) + GET_SIZE( ( (char*)( p - WSIZE ) ) ) )
#define GET_PREV(p)		( (char*)( p ) - GET_SIZE( ( (char*)( p - DSIZE ) ) ) )

#define GET_NEXT_FREE(p) 	GET_PTR_ADDR((unsigned int*)(p))
#define INSIDE_HEAP(p)		( (void*)p >= (void*)mem_hp && (void*)p < (void*)mem_bp )

//GLOBAL SCALARS
char *seg_lists;//ptr head of seg_lists table
char *mem_hp; 	//ptr head of heap
char *mem_bp;	//ptr end of heap

//METHOD DEFINITIONS
static void place(void* p, size_t size);
static int get_size_class(size_t size);
static void *get_fit(size_t size);
static void *grow_heap(size_t words);
static void coalesce(void *p, size_t size);
static void seg_list_remove(void *p);
static void seg_list_add(void *p);


/*
 * mm_init - initialize the malloc package. allocate enough space to
 * create empty seg_list table and one boundary block at head of heap.
 *
 * returns: 1 if successful, -1 on failure
 */
int mm_init( void )
{
  int seg_lists_size = SIZE_T_SIZE * SEG_LIST_COUNT;
  if( ( seg_lists = mem_sbrk( ALIGN( seg_lists_size ) + MIN_BLOCK_SIZE ) ) == ( void * ) -1 )
    return -1;

  int i;
  for( i = 0; i < SEG_LIST_COUNT; i++ )
    PUT( seg_lists + ( SIZE_T_SIZE * i ), 0 );

  mem_hp = (char*)( seg_lists + seg_lists_size );
  mem_hp = mem_hp + DSIZE;
  PUT( GET_HEADER( mem_hp ), PACK( MIN_BLOCK_SIZE, 1 ) );
  PUT( GET_FOOTER( mem_hp ), PACK( MIN_BLOCK_SIZE, 1 ) );
  mem_bp = mem_heap_hi();
  return 0;
}

/*
 * mm_malloc - allocate block of given size. implementation uses first first on seg_lists table.
 * when a free block is found, a basic split strategy looks to free up any used payload of alloc'd block
 * if no free block is found, the heap is extended by size.
 *
 * size_t size: size of alloc request
 *
 * returns: NULL if failure occurs, otherwise 8 byte ptr to address of allocated block's first payload byte.
 */
void *mm_malloc( size_t size )
{
  if( size == 0 )
    return NULL;

  void *fit_ptr;
  size_t block_size;

  block_size = ALIGN( size + DSIZE );

  if( block_size < MIN_BLOCK_SIZE )
    block_size = ALIGN( size + MIN_BLOCK_SIZE - size );

  if( ( fit_ptr = get_fit( block_size ) ) != NULL ){
    size_t split_remainder = GET_SIZE( GET_HEADER( fit_ptr ) ) - block_size;

    if( ( GET_SIZE( GET_HEADER( fit_ptr ) ) - block_size ) >= MIN_BLOCK_SIZE ){
      seg_list_remove( fit_ptr );
      place( fit_ptr, block_size );
      void* new_ptr = ( GET_SIZE( GET_HEADER( fit_ptr ) ) + fit_ptr );
      PUT( GET_HEADER( new_ptr ), PACK( split_remainder, 0 ) );
      PUT( GET_FOOTER( new_ptr ), PACK( split_remainder, 0 ) );
      seg_list_add( new_ptr );

    }else{
      place( fit_ptr, GET_SIZE( GET_HEADER( fit_ptr ) ) );
      seg_list_remove( fit_ptr );
    }

  } else if( ( fit_ptr = grow_heap( block_size ) ) != NULL ){
      place( fit_ptr, block_size );

  } else {
    return NULL;
  }
  return fit_ptr;
}

/*
 * mm_free - free block from ptr of first payload byte. implementation relies on coalesce. see
 * coalesce for more details.
 *
 *  * void* ptr: ptr to first byte of block's payload. ptr must not have already been freed
 *
 */
void mm_free( void *p )
{
  if ( p == NULL )
    return;

  size_t size = GET_SIZE( GET_HEADER( p ) );
  coalesce( p, size );
}

/*
 * mm_realloc - mm_realloc resizes block of ptr. if ptr is null, a new block is alloc'd.
 * if size is 0, the block of ptr is freed. otherwise block of ptr is resized to size.
 * returned ptr could be different then original in which case contents of original block
 * (up to size of new block) are copied. implementation is based on mm_alloc and mm_free.
 * see mm_alloc and mm_free for more detail.s
 *
 * void* ptr: ptr to first byte of block's payload.
 * size_t* size: desired block size.
 *
 * returns: 8 byte ptr to address of newly resized block
 *
 */
void *mm_realloc( void *ptr, size_t size )
{
  if ( ptr == NULL )
    return mm_malloc( size );
  if( size == 0 ){
    mm_free( ptr );
    return NULL;
  }

  void *old_ptr = ptr;
  void *new_ptr;
  size_t copySize;

  new_ptr = mm_malloc( size );
  if ( new_ptr == NULL )
    return NULL;
  copySize = GET_SIZE( GET_HEADER(ptr ) ) - DSIZE;

  if ( size < copySize )
    copySize = size;

  memcpy( new_ptr, old_ptr, copySize );

  mm_free( old_ptr );

  mem_bp = mem_heap_hi();
  return new_ptr;
}

/*
 * place - update block's header and footer data with alloc bit and size
 *
 * void* ptr: ptr to first byte of block's payload.
 * size_t* size: desired block size.
 *
 */
static void place( void* p, size_t size ){
  if ( p == NULL || size == 0 )
    return;

  PUT( GET_HEADER( p ), PACK( size, 1 ) );
  PUT( GET_FOOTER( p ), PACK( size, 1 ) );
}

/*
 * get_size_class - calculate size class lookup of seg_lists table
 * with naive hash strategy
 *
 * size_t* size: desired block size.
 *
 * returns: int, where 0 <= size class < SEG_LIST_COUNT
 */
static int get_size_class( size_t size )
{
  int class = size/64;
  if( class >= SEG_LIST_COUNT )
    return SEG_LIST_COUNT -1;
  else if ( class < 0 )
    return 0;
  else
    return class;
}

/*
 * get_fit - find free block of size from seg_lists table.
 * implementation based on first fit strategy, starting from
 * seg_list fit size class.
 *
 * size_t* size: desired block size.
 *
 * returns:  8 byte ptr to first payload byte address of fit block.
 */

static void* get_fit( size_t size )
{
  int i = get_size_class( size );

  while( i < SEG_LIST_COUNT ){
    void* j;

    for ( j = GET_PTR_ADDR( seg_lists + ( sizeof( size_t*)*i ) ); INSIDE_HEAP( j ) && j != 0; ) {

      if( !GET_ALLOC( GET_HEADER( j ) ) && GET_SIZE( GET_HEADER( j ) ) >= size )
        return j;

      j = GET_NEXT_FREE( j );
    }

    i++;
  }

  return NULL;
}

/*
 * grow_heap - extend heap by size and mark
 * new block as free.
 *
 * size_t* size: desired block size.
 *
 * returns: NULL if failure occurs, otherwise 8 byte ptr to address of allocated block's first payload byte.
 */
static void *grow_heap( size_t size )
{
  if( size == 0 )
    return NULL;

  void* ptr =  mem_sbrk( size );

  if( (long)ptr == -1 )
    return NULL;

  ptr = ptr + DSIZE;

  PUT( GET_HEADER( ptr ), PACK( size, 0 ) );
  PUT( GET_FOOTER( ptr ), PACK( size, 0 ) );

  mem_bp = mem_heap_hi();
  return ptr;

}

/*
 * coalesce - free block of ptr and of size. combine with neighboring blocks
 * if free.
 *
 * void* ptr: ptr to first byte of block's payload.
 * size_t* size: desired block size.
 *
 */

static void coalesce( void *p, size_t size )
{
  if ( p == NULL || size == 0 )
    return;

  int prev_elig = ( INSIDE_HEAP( GET_PREV( p ) ) && !GET_ALLOC( GET_HEADER( GET_PREV( p ) ) ) );
  int next_elig = ( INSIDE_HEAP( GET_NEXT( p ) ) && !GET_ALLOC( GET_HEADER( GET_NEXT( p ) ) ) );
  void* start_p = p;
  size_t free_size = size;

  if( prev_elig ){
    free_size += GET_SIZE( GET_HEADER( GET_PREV( p ) ) );
    seg_list_remove( GET_PREV( p ) );
    start_p = GET_PREV( p );
  }

  if( next_elig ){
    free_size += GET_SIZE( GET_HEADER( GET_NEXT( p ) ) );
    seg_list_remove( GET_NEXT( p ) );

  }

  PUT( GET_HEADER( start_p ), PACK( free_size, 0 ) );
  PUT( GET_FOOTER( start_p ), PACK( free_size, 0 ) );
  seg_list_add( start_p );
}

/*
 * seg_list_remove - remove free block from seg_lists table.
 *
 * void* ptr: ptr to first byte of block's payload.
 *
 */
static void seg_list_remove( void *p )
{

  int class_size = get_size_class( GET_SIZE( GET_HEADER( p ) ) );
  void* j;
  void* k = seg_lists + ( sizeof(void*) * class_size );


  for ( j = GET_PTR_ADDR( seg_lists + ( sizeof(size_t*) * class_size ) ); INSIDE_HEAP( j ) && j != 0; ) {

    if( p == j ){
      if( INSIDE_HEAP( GET_NEXT_FREE( j ) ) && GET_NEXT_FREE( j ) != 0 )
        PUT_PTR_ADDR( k, GET_NEXT_FREE( j ) );
      else
        PUT_PTR_ADDR( k, 0 );

      PUT_PTR_ADDR( p, 0 );
      return;
    }

    k = j;
    j = GET_NEXT_FREE( j );
  }
}

/*
 * seg_list_add - add free block to seg_lists table. block will be pushed
 * on top of stack for given size class
 *
 *  * void* ptr: ptr to first byte of block's payload.
 *
 */

static void seg_list_add( void *p )
{
  if ( p == NULL )
    return;
  int class_size = get_size_class( GET_SIZE( GET_HEADER( p ) ) );

  if( !!GET_PTR_ADDR( seg_lists + ( class_size * SIZE_T_SIZE ) ) 
    && GET_PTR_ADDR( seg_lists + ( class_size * SIZE_T_SIZE ) ) != p )
    
    PUT_PTR_ADDR(p,GET_PTR_ADDR(seg_lists +(class_size * SIZE_T_SIZE ) ));

  else
    PUT_PTR_ADDR( p, 0 );


  PUT_PTR_ADDR( seg_lists + ( class_size * SIZE_T_SIZE ), p );

}



