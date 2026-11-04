module.exports = {
  webpack: (config) => {
    config.module ??= {};
    config.module.rules ??= [];
    config.module.rules.push({
      test: /\.(png|jpe?g|gif|webp)$/i,
      type: "asset/inline",
    });
    return config;
  },
};
