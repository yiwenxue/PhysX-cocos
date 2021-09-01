let tick = 0

const entities = makeEntities()

const update = () => {
  tick++
  physics.update(entities)
  renderer.update(entities)
  requestAnimationFrame(update)
}

physics.onLoad(() => {
  renderer.init(entities)
  physics.init(entities)
  update()
})
