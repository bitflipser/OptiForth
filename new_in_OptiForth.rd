
new core words

0if ( -- )                      \  'dup 0= if'
  branch on processor Z-flag, no stack action!

0until ( -- )                   \  'dup 0= until'
  loop back on processor Z-flag, no stack action! small loops only!
  loop until Z-flag is set (e.g. checked bit is 0)

1until ( -- )                   \  'dup 0 <> until'
  loop back on processor Z-flag, no stack action! small loops only!
  loop until Z-flag is cleared (e.g. checked bit is 1)

mtst0 ( mask adr -- )
  check adr with mask, leaves no flag on stack, to be used with
  '0if', '0until' and '1until'

m@ ( adr -- n )
  (fast) direct fetch from RAM, do not use if 'adr' is a literal,
  constant or variable
  
mc@ ( adr -- c )
  (fast) direct fetch from RAM, do not use if 'adr' is a literal,
  constant or variable

swap- ( n1 n2 -- n2-n1 )        \ 'swap -'
  small and fast

1024*/ ( u1 u2 -- u3 )
  scale operator, multiply u1 and u2 with 32-bit intermediate,
  then divide by 1024 with rounding
  for scaling of ADC raw data or fixed point constant computation
  e.g. 'pi *' -> '3217 1024*/'

u*/ ( u1 u2 u3 -- u' )
  multiply u1 and u2 with 32-bit intermediate,
  then divide by u3 w/o rounding

u2/ ( u -- u' )                 \  '1 rshift'
  unsigned divide by 2

8<< ( u -- u' )                 \  '8 lshift'
  shift left by 8

8>> ( u -- u' )                 \  '8 rshift'
  shift right by 8

pick ( x1..xu..xn u -- x1..xu..xn xu )
  copy u-th value to TOP

sqr ( uc -- u' )                \  'dup *'
  square of unsigned value in TOP, uc < 256

sqrt ( u -- uc' )
  square root of unsigned value in TOP w/o rounding
  
ticks>n ( top -- ticks top )    \  'ticks swap'
  pushes the system ticks to NEXT

ticks= ( ticks -- ms )          \  'ticks swap -'
  leaves the time [ms] from ticks in TOP to the actual system ticks

us ( u -- )
  delay u microseconds, !!streched by interrupt routines!!

forget ( "name" -- )
  forget all the words back to "name"

gcd ( u1 u2 -- gcd )             \ for benchmarks only
  greatest common divider

do .. i .. j .. leave .. loop
!! one 'leave' per loop ONLY !!
!! in nested loops 'leave' MUST NOT be placed before the inner 'do' !!


