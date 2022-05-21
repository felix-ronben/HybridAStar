import matplotlib.pyplot as plt
import matplotlib.animation as ani
fig = plt.figure()
frame = []
artists = []
# plt.axis([0,20,0,20])
plt.plot([1,2,3,4,5],[5,4,3,2,1],'ob')
for i in range(20):
    frame = plt.plot(i,i,'or') + frame
    artists.append(frame)
    # plt.pause(0.01)
# plt.show()
[artists.append(frame) for i in range (10)]
animation = ani.ArtistAnimation(fig = fig, artists = artists)
animation.save('1.gif', fps = 30, writer='pillow')