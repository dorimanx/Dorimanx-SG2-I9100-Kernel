
My info guide.

How to cherry pick commits from other branch to yours.

fetch new branch, checkout inside.

git fetch git://github.com/dorimanx/Dorimanx-SG2-I9100-Kernel.git
git checkout FETCH_HEAD

now git log

find the hash number of the oldest commit that you want and the newst commit that you want,
it's a range of commits! :)

now example! ( the .. from old to new is a must)
rm -f *.patch  (clean all old junk)

you must use commit hash -1 from commit that you want to start adding!
just take commit that you have/not want as first!, git format-patch will start from next to one you put as the oldest.!

then "OLD" + "NEW" are info, dont write.

git format-patch "OLD" 177e5c7ce53b6d06b9ee3448c00215ba6d70ffc9..c87ade04d28d2024b8ed2000346aa568a07a7f0b "NEW"
git checkout DESTINATION/YOUR branch.
git am *.patch
rm -f *.patch
git push

all done :)

if git am fail to add patch, you can stop and try to fix the commit, that is another story :)

More will be added later.
